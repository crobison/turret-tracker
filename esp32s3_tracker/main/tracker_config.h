#pragma once
// =============================================================================
//  TRACKER SETTINGS - everything you are likely to tune lives in this file.
//  Edit, then:  idf.py build flash monitor
// =============================================================================

// ---------- Link to the turret's Arduino Nano ----------
// ESP32 GPIO that sends commands to the Nano's RX0 (D0) pin.
// GPIO14 is free on the common "ESP32-S3-CAM N16R8 dual USB-C" (Freenove-style)
// boards. If your board's silkscreen shows GPIO14 used/missing, pick another
// free pin: 1, 2, 3, 14, 21, 41, 42, 47 are usually free. Avoid 4-13, 15-18
// (camera), 19/20 (USB), 35-37 (PSRAM), 43/44 (console).
#define NANO_UART_TX_GPIO   14
#define NANO_UART_BAUD      9600      // must match Serial.begin() in the Nano sketch

// ---------- Camera ----------
// Pinout for Freenove-style ESP32-S3-CAM boards (same as Arduino's
// CAMERA_MODEL_ESP32S3_EYE). If camera init fails, check your seller's pin map.
#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK   15
#define CAM_PIN_SIOD    4
#define CAM_PIN_SIOC    5
#define CAM_PIN_D7     16   // Y9
#define CAM_PIN_D6     17   // Y8
#define CAM_PIN_D5     18   // Y7
#define CAM_PIN_D4     12   // Y6
#define CAM_PIN_D3     10   // Y5
#define CAM_PIN_D2      8   // Y4
#define CAM_PIN_D1      9   // Y3
#define CAM_PIN_D0     11   // Y2
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK   13

// Image orientation. The OV3660 on these boards usually comes out upside-down,
// so VFLIP=1 is the default (Espressif's CameraWebServer does the same).
// Check with the "which way is up" test in the guide and flip as needed.
#define CAM_VFLIP      1
#define CAM_HMIRROR    0

// esp32-camera delivers RGB565 with the high byte first (big-endian). If
// detection is oddly poor in good light, try 0 here.
#define CAM_RGB565_BIG_ENDIAN  1

// Frame is QVGA = 320 x 240.
#define FRAME_W        320
#define FRAME_H        240

// ---------- Detection ----------
#define DETECT_SCORE_THRESHOLD  0.60f   // model default is 0.70; lower = more detections, more false hits
#define MIN_BOX_HEIGHT_PX       40      // ignore tiny/far detections
#define TORSO_FRACTION          0.55f   // aim this far down the person box (0 = top of head, 1 = feet).
                                        // 0.55 = chest/belly. Do NOT lower toward the head.
#define REACQUIRE_RADIUS_PX     80      // stick with the same person if they moved less than this
#define LOST_AFTER_FRAMES       5       // forget the current target after this many empty frames

// ---------- Aim point ----------
// Pixel where darts actually land at typical range. Start at the image center,
// then calibrate (guide step 9): read "tgt=(x,y)" from the log while a target
// stands where the darts hit, and put those numbers here.
#define AIM_X   160
#define AIM_Y   120

// ---------- Control loop ----------
#define DEADBAND_X_PX           10      // "close enough" horizontally
#define DEADBAND_Y_PX           10      // "close enough" vertically

#define YAW_GAIN_MS_PER_PX      0.40f   // yaw pulse length per pixel of error
#define YAW_MIN_MS              20      // shortest pulse that actually moves the base
#define YAW_MAX_MS              150     // cap per step (Nano also caps at 250)
#define YAW_INVERT              0       // set 1 if the turret turns AWAY from the target

#define PITCH_GAIN_DEG_PER_PX   0.12f   // pitch degrees per pixel of error
#define PITCH_MAX_DEG           8       // cap per step (Nano also caps at 15)
#define PITCH_INVERT            0       // set 1 if the barrel tilts AWAY from the target

#define SETTLE_MS               120     // wait after a move so the next frame isn't motion-blurred

#define LOCK_FRAMES_TO_FIRE     3       // consecutive on-target frames before sending "F"
                                        // (the Nano still ignores F unless auto-fire is armed)

// Set to 0 to run detection + logging only, with no commands sent to the Nano.
// Useful for the first bring-up.
#define SEND_COMMANDS           0       // START AT 0. Set to 1 once the direction checks pass (guide step 8).

// ---------- Live view (Wi-Fi) ----------
// Web page with the camera stream and detection boxes:
//     http://turret.local/   (or the IP address printed in the serial log)
// Wi-Fi name/password go in main/wifi_secrets.h (created on first build,
// git-ignored). With no SSID set, the tracker runs without Wi-Fi.
#define ENABLE_WEB_VIEW         1
#define WEB_HOSTNAME            "turret"   // -> http://turret.local/
#define WEB_JPEG_QUALITY        60         // 1-100; higher = sharper but slower/bigger
