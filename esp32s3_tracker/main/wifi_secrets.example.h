#pragma once
// Wi-Fi credentials for the live view.
//
// The build copies this file to wifi_secrets.h the first time you build.
// Edit main/wifi_secrets.h (NOT this example file) and rebuild.
// wifi_secrets.h is listed in .gitignore so your password never gets committed.
//
// Leave WIFI_SSID empty to run without Wi-Fi (tracking still works, no live view).
// The ESP32-S3 only supports 2.4 GHz Wi-Fi networks.

#define WIFI_SSID     ""
#define WIFI_PASSWORD ""
