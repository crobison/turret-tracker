# First build: create wifi_secrets.h from the example so the build works out of
# the box. Put your real Wi-Fi name/password in wifi_secrets.h (git-ignored).
if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/wifi_secrets.h")
    configure_file("${CMAKE_CURRENT_LIST_DIR}/wifi_secrets.example.h"
                   "${CMAKE_CURRENT_LIST_DIR}/wifi_secrets.h" COPYONLY)
    message(WARNING "Created main/wifi_secrets.h from the example. "
                    "Fill in WIFI_SSID / WIFI_PASSWORD there to enable the live view.")
endif()

idf_component_register(SRCS "app_main.cpp" "web_view.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES pedestrian_detect esp32-camera mdns
                                esp_driver_uart esp_timer esp_wifi esp_netif esp_event
                                esp_http_server nvs_flash)
