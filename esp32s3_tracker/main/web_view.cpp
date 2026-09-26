#include "web_view.hpp"

#include <atomic>
#include <cstdio>
#include <cstring>

#include "esp_camera.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "img_converters.h"
#include "mdns.h"
#include "nvs_flash.h"

#include "tracker_config.h"
#include "wifi_secrets.h" // created from wifi_secrets.example.h on first build; not committed to git

static const char *TAG = "web";

#if ENABLE_WEB_VIEW

// ---------------------------------------------------------------------------
// Shared state between the tracker task (writer) and the web server tasks
// ---------------------------------------------------------------------------
static SemaphoreHandle_t s_frame_lock;  // guards s_frame
static SemaphoreHandle_t s_status_lock; // guards s_status
static uint8_t *s_frame = nullptr;      // annotated RGB565 copy, PSRAM
static int s_w = 0, s_h = 0;
static bool s_have_frame = false;
static std::atomic<uint32_t> s_seq{0};
static std::atomic<int> s_viewers{0};
static TrackerStatus s_status;
static char s_ip[16] = "";
static bool s_started = false;

// ---------------------------------------------------------------------------
// Overlay drawing (directly on RGB565 pixels)
// ---------------------------------------------------------------------------
static constexpr uint16_t COL_GREEN = 0x07E0;  // chosen target
static constexpr uint16_t COL_YELLOW = 0xFFE0; // other detections
static constexpr uint16_t COL_RED = 0xF800;    // aim crosshair
static constexpr uint16_t COL_CYAN = 0x07FF;   // torso point
static constexpr uint16_t COL_WHITE = 0xFFFF;  // lock indicator

static inline void put_px(int x, int y, uint16_t c)
{
    if ((unsigned)x >= (unsigned)s_w || (unsigned)y >= (unsigned)s_h) {
        return;
    }
    uint8_t *p = s_frame + ((size_t)y * s_w + x) * 2;
#if CAM_RGB565_BIG_ENDIAN
    p[0] = (uint8_t)(c >> 8);
    p[1] = (uint8_t)(c & 0xFF);
#else
    p[0] = (uint8_t)(c & 0xFF);
    p[1] = (uint8_t)(c >> 8);
#endif
}

static void hline(int x1, int x2, int y, uint16_t c)
{
    for (int x = x1; x <= x2; x++) {
        put_px(x, y, c);
    }
}

static void vline(int x, int y1, int y2, uint16_t c)
{
    for (int y = y1; y <= y2; y++) {
        put_px(x, y, c);
    }
}

static void rect(int x1, int y1, int x2, int y2, uint16_t c, int thickness)
{
    for (int t = 0; t < thickness; t++) {
        hline(x1 + t, x2 - t, y1 + t, c);
        hline(x1 + t, x2 - t, y2 - t, c);
        vline(x1 + t, y1 + t, y2 - t, c);
        vline(x2 - t, y1 + t, y2 - t, c);
    }
}

static void fill_rect(int x1, int y1, int x2, int y2, uint16_t c)
{
    for (int y = y1; y <= y2; y++) {
        hline(x1, x2, y, c);
    }
}

static void draw_overlay(const OverlayBox *boxes, int n, const TrackerStatus &st)
{
    // Aim point: crosshair plus the deadband ("close enough") rectangle.
    uint16_t aim_col = st.lock ? COL_WHITE : COL_RED;
    rect(AIM_X - DEADBAND_X_PX, AIM_Y - DEADBAND_Y_PX, AIM_X + DEADBAND_X_PX, AIM_Y + DEADBAND_Y_PX, aim_col, 1);
    hline(AIM_X - 22, AIM_X - DEADBAND_X_PX - 3, AIM_Y, aim_col);
    hline(AIM_X + DEADBAND_X_PX + 3, AIM_X + 22, AIM_Y, aim_col);
    vline(AIM_X, AIM_Y - 22, AIM_Y - DEADBAND_Y_PX - 3, aim_col);
    vline(AIM_X, AIM_Y + DEADBAND_Y_PX + 3, AIM_Y + 22, aim_col);

    // Detections: other people thin yellow, chosen target thick green.
    for (int i = 0; i < n; i++) {
        const OverlayBox &b = boxes[i];
        if (b.chosen) {
            rect(b.x1, b.y1, b.x2, b.y2, COL_GREEN, 3);
        } else {
            rect(b.x1, b.y1, b.x2, b.y2, COL_YELLOW, 1);
        }
    }

    // Torso point being steered to the crosshair.
    if (st.has_target) {
        fill_rect(st.tx - 3, st.ty - 3, st.tx + 3, st.ty + 3, COL_CYAN);
    }

    // Red flash border on frames where a fire request was sent.
    if (st.fire) {
        rect(0, 0, s_w - 1, s_h - 1, COL_RED, 4);
    }
}

// ---------------------------------------------------------------------------
// Web page
// ---------------------------------------------------------------------------
static const char INDEX_HTML[] = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Turret Live View</title>
<style>
:root{--bg:#111418;--panel:#1b2027;--text:#e6e9ee;--muted:#9aa4b2;--green:#3ddc84;--yellow:#ffd84d;--red:#ff5a5a;--cyan:#4de3ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.4 system-ui,sans-serif}
main{max-width:980px;margin:0 auto;padding:12px 16px 32px}
h1{font-size:18px;margin:4px 0 12px}
.cam{position:relative;background:#000;border-radius:8px;overflow:hidden;aspect-ratio:4/3}
.cam img{width:100%;height:100%;display:block;image-rendering:pixelated}
.badge{position:absolute;top:10px;left:10px;padding:4px 10px;border-radius:6px;font-weight:600;background:rgba(0,0,0,.65)}
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:8px;margin-top:12px}
.stat{background:var(--panel);border-radius:8px;padding:8px 10px}.stat b{display:block;font-size:18px}
.stat span{color:var(--muted);font-size:12px}
.legend{display:flex;flex-wrap:wrap;gap:14px;margin-top:12px;color:var(--muted);font-size:13px}
.sw{display:inline-block;width:12px;height:12px;border-radius:2px;margin-right:6px;vertical-align:-1px}
p.note{color:var(--muted);font-size:13px}
</style></head><body><main>
<h1>Turret Live View</h1>
<div class="cam"><img id="cam" alt="camera stream"><div class="badge" id="badge">connecting…</div></div>
<div class="stats">
 <div class="stat"><b id="fps">–</b><span>frames / sec</span></div>
 <div class="stat"><b id="inf">–</b><span>detector ms</span></div>
 <div class="stat"><b id="det">–</b><span>people seen</span></div>
 <div class="stat"><b id="tgt">–</b><span>target point x, y (px)</span></div>
 <div class="stat"><b id="err">–</b><span>aim error x, y (px)</span></div>
 <div class="stat"><b id="cmd">–</b><span>last command</span></div>
 <div class="stat"><b id="fires">–</b><span>fire requests</span></div>
</div>
<div class="legend">
 <span><i class="sw" style="background:var(--green)"></i>target</span>
 <span><i class="sw" style="background:var(--yellow)"></i>other people</span>
 <span><i class="sw" style="background:var(--cyan)"></i>torso aim point</span>
 <span><i class="sw" style="background:var(--red)"></i>crosshair (turns white on lock)</span>
</div>
<p class="note" id="note"></p>
</main>
<script>
const cam=document.getElementById('cam');
function startStream(){cam.src='http://'+location.hostname+':81/stream?'+Date.now();}
cam.onerror=()=>setTimeout(startStream,1500);
startStream();
const $=id=>document.getElementById(id);
async function poll(){
 try{
  const s=await (await fetch('/status',{cache:'no-store'})).json();
  $('fps').textContent=s.loop_ms>0?(1000/s.loop_ms).toFixed(1):'–';
  $('inf').textContent=s.infer_ms;
  $('det').textContent=s.n_det;
  $('tgt').textContent=s.has_target?(s.tx+', '+s.ty):'–';
  $('err').textContent=s.has_target?(s.ex+', '+s.ey):'–';
  $('cmd').textContent='P'+(s.pitch_cmd>=0?'+':'')+s.pitch_cmd+'  Y'+(s.yaw_cmd>=0?'+':'')+s.yaw_cmd;
  $('fires').textContent=s.fire_requests;
  const b=$('badge');
  if(s.fire){b.textContent='FIRE REQUEST';b.style.color='var(--red)';}
  else if(s.lock){b.textContent='LOCKED';b.style.color='#fff';}
  else if(s.has_target){b.textContent='TRACKING';b.style.color='var(--green)';}
  else{b.textContent='no target';b.style.color='var(--muted)';}
  $('note').textContent=(s.send_commands?'Commands to turret: ON.':'Commands to turret: OFF (SEND_COMMANDS 0, watch-only).')+
   ' Tracking on/off (remote 1) and auto-fire arming (remote #) live on the turret, so this page cannot see them.';
 }catch(e){$('badge').textContent='reconnecting…';}
 setTimeout(poll,300);
}
poll();
</script></body></html>)HTML";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    TrackerStatus st;
    xSemaphoreTake(s_status_lock, portMAX_DELAY);
    st = s_status;
    xSemaphoreGive(s_status_lock);

    char json[400];
    snprintf(json,
             sizeof(json),
             "{\"frame\":%lu,\"infer_ms\":%d,\"loop_ms\":%d,\"n_det\":%d,\"has_target\":%s,"
             "\"tx\":%d,\"ty\":%d,\"ex\":%d,\"ey\":%d,\"pitch_cmd\":%d,\"yaw_cmd\":%d,"
             "\"lock\":%s,\"fire\":%s,\"fire_requests\":%lu,\"send_commands\":%s,"
             "\"aim_x\":%d,\"aim_y\":%d,\"viewers\":%d,\"ip\":\"%s\"}",
             (unsigned long)st.frame_no,
             st.infer_ms,
             st.loop_ms,
             st.n_det,
             st.has_target ? "true" : "false",
             st.tx,
             st.ty,
             st.ex,
             st.ey,
             st.pitch_cmd,
             st.yaw_cmd,
             st.lock ? "true" : "false",
             st.fire ? "true" : "false",
             (unsigned long)st.fire_requests,
             SEND_COMMANDS ? "true" : "false",
             AIM_X,
             AIM_Y,
             s_viewers.load(),
             s_ip);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

#define PART_BOUNDARY "turretframeboundary7d3f"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (res != ESP_OK) {
        return res;
    }

    s_viewers++;
    ESP_LOGI(TAG, "Viewer connected (%d watching)", s_viewers.load());
    uint32_t last_seq = 0;

    while (res == ESP_OK) {
        // Wait (up to 2 s) for the tracker to publish a new annotated frame.
        for (int waited = 0; s_seq.load() == last_seq && waited < 2000; waited += 20) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        uint8_t *jpg = nullptr;
        size_t jpg_len = 0;
        bool ok = false;
        xSemaphoreTake(s_frame_lock, portMAX_DELAY);
        last_seq = s_seq.load();
        if (s_have_frame) {
            ok = fmt2jpg(s_frame, (size_t)s_w * s_h * 2, s_w, s_h, PIXFORMAT_RGB565, WEB_JPEG_QUALITY, &jpg, &jpg_len);
        }
        xSemaphoreGive(s_frame_lock);

        if (!ok) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        char part[64];
        int hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned)jpg_len);
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, part, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)jpg, jpg_len);
        }
        free(jpg);
    }

    s_viewers--;
    ESP_LOGI(TAG, "Viewer disconnected (%d watching)", s_viewers.load());
    return res;
}

static void start_http_servers()
{
    // Page + status on port 80. Both servers run on core 0 at low priority,
    // leaving core 1 to the detector.
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32768;
    cfg.core_id = 0;
    cfg.task_priority = 3;
    cfg.stack_size = 6144;
    cfg.max_open_sockets = 4;
    cfg.lru_purge_enable = true;
    httpd_handle_t web = nullptr;
    if (httpd_start(&web, &cfg) == ESP_OK) {
        httpd_uri_t index = {};
        index.uri = "/";
        index.method = HTTP_GET;
        index.handler = index_handler;
        httpd_register_uri_handler(web, &index);

        httpd_uri_t status = {};
        status.uri = "/status";
        status.method = HTTP_GET;
        status.handler = status_handler;
        httpd_register_uri_handler(web, &status);
    } else {
        ESP_LOGE(TAG, "Could not start web server on port 80");
    }

    // The MJPEG stream never ends, so it gets its own server on port 81
    // (otherwise it would block the page and /status).
    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port = 81;
    scfg.ctrl_port = 32769;
    scfg.core_id = 0;
    scfg.task_priority = 3;
    scfg.stack_size = 8192;
    scfg.max_open_sockets = 3;
    scfg.lru_purge_enable = true;
    httpd_handle_t stream = nullptr;
    if (httpd_start(&stream, &scfg) == ESP_OK) {
        httpd_uri_t s = {};
        s.uri = "/stream";
        s.method = HTTP_GET;
        s.handler = stream_handler;
        httpd_register_uri_handler(stream, &s);
    } else {
        ESP_LOGE(TAG, "Could not start stream server on port 81");
    }
}

// ---------------------------------------------------------------------------
// Wi-Fi
// ---------------------------------------------------------------------------
static void wifi_event_handler(void *, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        auto *d = static_cast<wifi_event_sta_disconnected_t *>(data);
        ESP_LOGW(TAG, "Wi-Fi disconnected (reason %d), retrying...", d ? d->reason : -1);
        s_ip[0] = '\0';
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto *e = static_cast<ip_event_got_ip_t *>(data);
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "==================================================");
        ESP_LOGI(TAG, " Live view:  http://%s.local/   or   http://%s/", WEB_HOSTNAME, s_ip);
        ESP_LOGI(TAG, "==================================================");
    }
}

static bool start_wifi()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    esp_netif_set_hostname(netif, WEB_HOSTNAME);

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, nullptr));

    wifi_config_t wc = {};
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid));
    strncpy((char *)wc.sta.password, WIFI_PASSWORD, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = strlen(WIFI_PASSWORD) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE); // smoother streaming; costs a little power

    if (mdns_init() == ESP_OK) {
        mdns_hostname_set(WEB_HOSTNAME);
        mdns_instance_name_set("Turret Live View");
        mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
    } else {
        ESP_LOGW(TAG, "mDNS failed to start; use the IP address instead of %s.local", WEB_HOSTNAME);
    }
    ESP_LOGI(TAG, "Connecting to Wi-Fi \"%s\"...", WIFI_SSID);
    return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
void web_view_start()
{
    if (strlen(WIFI_SSID) == 0) {
        ESP_LOGW(TAG, "Live view OFF: set WIFI_SSID / WIFI_PASSWORD in main/wifi_secrets.h and rebuild.");
        return;
    }

    s_frame_lock = xSemaphoreCreateMutex();
    s_status_lock = xSemaphoreCreateMutex();
    s_w = FRAME_W;
    s_h = FRAME_H;
    s_frame = (uint8_t *)heap_caps_malloc((size_t)s_w * s_h * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_frame) {
        ESP_LOGE(TAG, "No memory for the live-view frame buffer; live view disabled");
        return;
    }
    jpgSetRgb565BE(CAM_RGB565_BIG_ENDIAN);

    start_wifi();
    start_http_servers();
    s_started = true;
}

void web_view_publish(const uint8_t *rgb565, int width, int height, const OverlayBox *boxes, int n_boxes,
                      const TrackerStatus &status)
{
    if (!s_started) {
        return;
    }

    xSemaphoreTake(s_status_lock, portMAX_DELAY);
    s_status = status;
    xSemaphoreGive(s_status_lock);

    if (s_viewers.load() == 0 || width != s_w || height != s_h) {
        return;
    }
    // If a stream is busy encoding the previous frame, skip this one rather
    // than make the tracker wait.
    if (xSemaphoreTake(s_frame_lock, 0) != pdTRUE) {
        return;
    }
    memcpy(s_frame, rgb565, (size_t)width * height * 2);
    draw_overlay(boxes, n_boxes, status);
    s_have_frame = true;
    s_seq++;
    xSemaphoreGive(s_frame_lock);
}

#else // ENABLE_WEB_VIEW == 0

void web_view_start()
{
    ESP_LOGI(TAG, "Live view disabled (ENABLE_WEB_VIEW 0)");
}

void web_view_publish(const uint8_t *, int, int, const OverlayBox *, int, const TrackerStatus &) {}

#endif
