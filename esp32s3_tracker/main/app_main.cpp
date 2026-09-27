// ESP32-S3 vision co-processor for the CrunchLabs Hack Pack IR Turret.
//
// Also serves a Wi-Fi live view with the detections drawn on (web_view.cpp).
//
// Loop: grab a 320x240 frame -> run Espressif's on-device pedestrian detector
// (ESP-DL PicoDet, ~130 ms on ESP32-S3) -> pick one person -> compute the pixel
// error between their torso and the calibrated aim point -> send short text
// commands to the turret's Arduino Nano over UART:
//
//     "P+3\n"  pitch up 3 deg      "Y-40\n" yaw left 40 ms      "F\n" fire request
//
// The Nano decides whether to obey (tracking must be switched on with remote
// button 1, and F is ignored unless auto-fire is armed with #).
//
// All tunables are in tracker_config.h.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <list>

#include "driver/uart.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pedestrian_detect.hpp"
#include "tracker_config.h"
#include "web_view.hpp"

static const char *TAG = "tracker";
static constexpr uart_port_t NANO_UART = UART_NUM_1;

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------
static esp_err_t init_camera()
{
    camera_config_t cfg = {};
    cfg.pin_pwdn = CAM_PIN_PWDN;
    cfg.pin_reset = CAM_PIN_RESET;
    cfg.pin_xclk = CAM_PIN_XCLK;
    cfg.pin_sccb_sda = CAM_PIN_SIOD;
    cfg.pin_sccb_scl = CAM_PIN_SIOC;
    cfg.pin_d7 = CAM_PIN_D7;
    cfg.pin_d6 = CAM_PIN_D6;
    cfg.pin_d5 = CAM_PIN_D5;
    cfg.pin_d4 = CAM_PIN_D4;
    cfg.pin_d3 = CAM_PIN_D3;
    cfg.pin_d2 = CAM_PIN_D2;
    cfg.pin_d1 = CAM_PIN_D1;
    cfg.pin_d0 = CAM_PIN_D0;
    cfg.pin_vsync = CAM_PIN_VSYNC;
    cfg.pin_href = CAM_PIN_HREF;
    cfg.pin_pclk = CAM_PIN_PCLK;
    cfg.xclk_freq_hz = 20000000;
    cfg.ledc_timer = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.pixel_format = PIXFORMAT_RGB565; // raw pixels; the detector resizes to 224x224 itself
    cfg.frame_size = FRAMESIZE_QVGA;     // 320x240
    cfg.jpeg_quality = 12;               // unused for RGB565
    cfg.fb_count = 2;
    cfg.fb_location = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode = CAMERA_GRAB_LATEST;  // always hand us the newest frame

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        ESP_LOGI(TAG, "Camera sensor PID 0x%04x (OV3660 = 0x3660)", s->id.PID);
        s->set_vflip(s, CAM_VFLIP);
        s->set_hmirror(s, CAM_HMIRROR);
        if (s->id.PID == OV3660_PID) {
            // Same tweaks Espressif's CameraWebServer applies to the OV3660.
            s->set_brightness(s, 1);
            s->set_saturation(s, -2);
        }
    }
    return ESP_OK;
}

// Throw away a frame so the next one is captured after the turret stopped moving.
static void discard_one_frame()
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

// ---------------------------------------------------------------------------
// UART link to the Nano (TX only; we never connect the Nano's 5 V TX to us)
// ---------------------------------------------------------------------------
static void init_nano_uart()
{
    uart_config_t uc = {};
    uc.baud_rate = NANO_UART_BAUD;
    uc.data_bits = UART_DATA_8_BITS;
    uc.parity = UART_PARITY_DISABLE;
    uc.stop_bits = UART_STOP_BITS_1;
    uc.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uc.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(NANO_UART, 256, 256, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(NANO_UART, &uc));
    ESP_ERROR_CHECK(
        uart_set_pin(NANO_UART, NANO_UART_TX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void send_to_nano(const char *line)
{
#if SEND_COMMANDS
    uart_write_bytes(NANO_UART, line, strlen(line));
    uart_wait_tx_done(NANO_UART, pdMS_TO_TICKS(50));
#else
    (void)line;
#endif
}

// ---------------------------------------------------------------------------
// Target selection
// ---------------------------------------------------------------------------
struct Target {
    bool valid = false;
    int tx = 0, ty = 0;                 // aim point on the person (torso), pixels
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0; // person box
    int box_h = 0;
    float score = 0.f;
};

// Prefer the person we were already following; otherwise take the biggest (closest) one.
static Target pick_target(const std::list<dl::detect::result_t> &results, const Target &prev, bool have_prev)
{
    Target best;
    long best_metric = -1;

    for (const auto &r : results) {
        if (r.box.size() < 4) {
            continue;
        }
        int x1 = r.box[0], y1 = r.box[1], x2 = r.box[2], y2 = r.box[3];
        int w = x2 - x1, h = y2 - y1;
        if (h < MIN_BOX_HEIGHT_PX || w <= 0) {
            continue;
        }
        int tx = (x1 + x2) / 2;
        int ty = y1 + (int)lroundf(TORSO_FRACTION * (float)h);

        long metric;
        if (have_prev) {
            long dx = tx - prev.tx, dy = ty - prev.ty;
            long d2 = dx * dx + dy * dy;
            if (d2 > (long)REACQUIRE_RADIUS_PX * REACQUIRE_RADIUS_PX) {
                continue;
            }
            metric = 1000000L - d2; // closer to previous = better
        } else {
            metric = (long)w * h; // bigger = closer = better
        }

        if (metric > best_metric) {
            best_metric = metric;
            best.valid = true;
            best.tx = tx;
            best.ty = ty;
            best.x1 = x1;
            best.y1 = y1;
            best.x2 = x2;
            best.y2 = y2;
            best.box_h = h;
            best.score = r.score;
        }
    }

    // Previous person vanished but someone else is in view: fall back to biggest.
    if (!best.valid && have_prev && !results.empty()) {
        return pick_target(results, prev, false);
    }
    return best;
}

static int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// ---------------------------------------------------------------------------
// Tracking loop (runs on core 1; Wi-Fi and the web server use core 0)
// ---------------------------------------------------------------------------
static constexpr int MAX_OVERLAY_BOXES = 10;

static void tracker_task(void *)
{
    ESP_LOGI(TAG, "Loading pedestrian detection model...");
    PedestrianDetect *detect = new PedestrianDetect(PedestrianDetect::PICO_S8_V1, false);
    detect->set_score_thr(DETECT_SCORE_THRESHOLD, 0);
    ESP_LOGI(TAG, "Ready. Aim point = (%d,%d). Commands to Nano: %s", AIM_X, AIM_Y, SEND_COMMANDS ? "ON" : "OFF");

    Target prev;
    int frames_since_seen = 1000;
    int lock_count = 0;
    uint32_t frame_no = 0;
    uint32_t fire_requests = 0;
    int64_t t_prev = esp_timer_get_time();

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Frame grab failed");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        frame_no++;

        int64_t t0 = esp_timer_get_time();
        dl::image::img_t img = {};
        img.data = fb->buf;
        img.width = (uint16_t)fb->width;
        img.height = (uint16_t)fb->height;
        img.pix_type = CAM_RGB565_BIG_ENDIAN ? dl::image::DL_IMAGE_PIX_TYPE_RGB565BE
                                             : dl::image::DL_IMAGE_PIX_TYPE_RGB565LE;

        std::list<dl::detect::result_t> &results = detect->run(img);
        int infer_ms = (int)((esp_timer_get_time() - t0) / 1000);
        int loop_ms = (int)((t0 - t_prev) / 1000);
        t_prev = t0;

        bool have_prev = prev.valid && frames_since_seen < LOST_AFTER_FRAMES;
        Target tgt = pick_target(results, prev, have_prev);
        int n_det = (int)results.size();

        TrackerStatus st;
        st.frame_no = frame_no;
        st.infer_ms = infer_ms;
        st.loop_ms = loop_ms;
        st.n_det = n_det;

        // Boxes for the live view.
        OverlayBox boxes[MAX_OVERLAY_BOXES];
        int n_boxes = 0;
        for (const auto &r : results) {
            if (n_boxes >= MAX_OVERLAY_BOXES || r.box.size() < 4) {
                continue;
            }
            OverlayBox &b = boxes[n_boxes++];
            b.x1 = (int16_t)r.box[0];
            b.y1 = (int16_t)r.box[1];
            b.x2 = (int16_t)r.box[2];
            b.y2 = (int16_t)r.box[3];
            b.score = r.score;
            b.chosen = tgt.valid && r.box[0] == tgt.x1 && r.box[1] == tgt.y1 && r.box[2] == tgt.x2 &&
                r.box[3] == tgt.y2;
        }

        if (!tgt.valid) {
            frames_since_seen++;
            lock_count = 0;
            ESP_LOGI(TAG, "%3d ms | %d det | no target", infer_ms, n_det);
            web_view_publish(fb->buf, fb->width, fb->height, boxes, n_boxes, st);
            esp_camera_fb_return(fb);
            vTaskDelay(1);
            continue;
        }
        frames_since_seen = 0;
        prev = tgt;

        int ex = tgt.tx - AIM_X; // + = target is right of the aim point
        int ey = tgt.ty - AIM_Y; // + = target is below the aim point

        // Yaw: pulse length proportional to horizontal error. + = turn right.
        int yaw_ms = 0;
        if (abs(ex) > DEADBAND_X_PX) {
            int mag = clampi((int)lroundf(YAW_GAIN_MS_PER_PX * (float)abs(ex)), YAW_MIN_MS, YAW_MAX_MS);
            yaw_ms = (ex > 0 ? mag : -mag) * (YAW_INVERT ? -1 : 1);
        }

        // Pitch: degrees proportional to vertical error. + = aim up (target above aim point).
        // If the box is cut off at the top or bottom of the frame, ty no longer moves when the
        // barrel tilts (a full-height box always gives the same ty), so pitch would run away to
        // its limit. Hold pitch until the whole person is back in view.
        bool box_clipped_y = tgt.y1 <= 2 || tgt.y2 >= (int)fb->height - 3;
        int pitch_deg = 0;
        if (abs(ey) > DEADBAND_Y_PX && !box_clipped_y) {
            int mag = clampi((int)lroundf(PITCH_GAIN_DEG_PER_PX * (float)abs(ey)), 1, PITCH_MAX_DEG);
            pitch_deg = (ey < 0 ? mag : -mag) * (PITCH_INVERT ? -1 : 1);
        }

        bool on_target = (yaw_ms == 0 && pitch_deg == 0);
        lock_count = on_target ? lock_count + 1 : 0;

        char line[16];
        if (pitch_deg != 0) {
            snprintf(line, sizeof(line), "P%+d\n", pitch_deg);
            send_to_nano(line); // send pitch first: the Nano applies it instantly
        }
        if (yaw_ms != 0) {
            snprintf(line, sizeof(line), "Y%+d\n", yaw_ms);
            send_to_nano(line); // the Nano blocks for yaw_ms while turning
        }

        bool fire = false;
        if (lock_count >= LOCK_FRAMES_TO_FIRE) {
            send_to_nano("F\n"); // ignored by the Nano unless auto-fire is armed
            fire = true;
            fire_requests++;
            lock_count = 0;
        }

        st.has_target = true;
        st.tx = tgt.tx;
        st.ty = tgt.ty;
        st.ex = ex;
        st.ey = ey;
        st.pitch_cmd = pitch_deg;
        st.yaw_cmd = yaw_ms;
        st.lock = on_target;
        st.fire = fire;
        st.fire_requests = fire_requests;
        web_view_publish(fb->buf, fb->width, fb->height, boxes, n_boxes, st);
        esp_camera_fb_return(fb);

        ESP_LOGI(TAG,
                 "%3d ms | %d det | tgt=(%d,%d) h=%d s=%.2f | err=(%+d,%+d) | P%+d Y%+d%s%s",
                 infer_ms,
                 n_det,
                 tgt.tx,
                 tgt.ty,
                 tgt.box_h,
                 tgt.score,
                 ex,
                 ey,
                 pitch_deg,
                 yaw_ms,
                 on_target ? " | LOCK" : "",
                 fire ? " | FIRE" : "");

        if (yaw_ms != 0 || pitch_deg != 0) {
            // Let the turret finish moving and stop shaking, then drop the stale frame.
            vTaskDelay(pdMS_TO_TICKS(abs(yaw_ms) + SETTLE_MS));
            discard_one_frame();
        } else {
            vTaskDelay(1);
        }
    }
}

// ---------------------------------------------------------------------------
// Startup
// ---------------------------------------------------------------------------
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Turret tracker starting");

    esp_err_t err = init_camera();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed (0x%x). Check CAM_PIN_* in tracker_config.h and the ribbon cable.", err);
        return;
    }
    init_nano_uart();
    web_view_start(); // Wi-Fi live view (skipped if no SSID is set)

    xTaskCreatePinnedToCore(tracker_task, "tracker", 12 * 1024, nullptr, 5, nullptr, 1);
}
