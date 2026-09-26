#pragma once
// Wi-Fi live view: a web page with the camera stream and the tracker's
// detections drawn on top.
//
//   http://turret.local/      (or the IP address printed in the serial log)
//
// Port 80 serves the page and /status (JSON). Port 81 serves /stream (MJPEG).
// JPEG encoding only happens while a browser is watching, and it runs on the
// other CPU core from the detector, so tracking barely slows down.

#include <cstdint>

struct OverlayBox {
    int16_t x1, y1, x2, y2;
    float score;
    bool chosen; // the person the turret is steering toward
};

struct TrackerStatus {
    uint32_t frame_no = 0;
    int infer_ms = 0;     // detector time for this frame
    int loop_ms = 0;      // full loop time (1000 / loop_ms = frames per second)
    int n_det = 0;        // people detected in this frame
    bool has_target = false;
    int tx = 0, ty = 0;   // torso aim point on the target, pixels
    int ex = 0, ey = 0;   // error from the calibrated aim point, pixels
    int pitch_cmd = 0;    // last pitch command sent (degrees, + = up)
    int yaw_cmd = 0;      // last yaw command sent (ms, + = right)
    bool lock = false;    // inside the deadband this frame
    bool fire = false;    // "F" sent this frame
    uint32_t fire_requests = 0;
};

// Connects to Wi-Fi and starts the web servers. Does nothing if the live view
// is disabled in tracker_config.h or no Wi-Fi name is set in wifi_secrets.h.
void web_view_start();

// Called by the tracker once per frame. Always updates /status. Copies the
// frame and draws the overlay only when a browser is watching.
void web_view_publish(const uint8_t *rgb565, int width, int height, const OverlayBox *boxes, int n_boxes,
                      const TrackerStatus &status);
