# Turret Tracker

Camera-based target tracking for the **CrunchLabs Hack Pack IR Turret**, using an **ESP32-S3-CAM (N16R8, OV3660)** as a vision co-processor.

- The ESP32-S3 runs Espressif's on-device person detector at about 7 frames per second.
- It steers the turret so the target's torso lines up with a calibrated crosshair.
- It serves a **Wi-Fi live view**: detection boxes, the aim point and tracking stats, visible from any phone or laptop on your network.
- The turret's own Arduino Nano still drives the servos and still obeys the IR remote. Auto-fire has to be armed from the remote, and the Nano enforces the safety limits.

```
 OV3660 ─► ESP32-S3-CAM (mounted on the barrel)          Arduino Nano (turret base, stock board)
           • 320x240 frames                               • drives yaw / pitch / roll servos
           • ESP-DL pedestrian detector (~130 ms)         • IR remote works as before
           • picks a person, aims at the torso            • NEW: obeys "Y+40" "P-3" "F" on RX0 (D0)
           • sends commands ──── GPIO14 ────────────────► • NEW: remote 1 = tracking on/off
           • Wi-Fi live view      GND ─────────────────── • NEW: remote # = auto-fire arm/disarm
               http://turret.local/
```

> **Status.** Both programs compile cleanly: the Nano sketch with avr-gcc and IRremote 4.4.1, the ESP32 firmware with ESP-IDF v5.5.1. The Nano's command and safety logic has unit tests that run on a PC, and the live-view page was checked in a browser against a simulated turret. Real-hardware tuning (directions, gains, aim point) is covered in Parts F–H.

---

## Contents

- [Repository layout](#repository-layout)
- [Quick reference](#quick-reference)
- [Parts](#parts)
- [Part A: One-time computer setup](#part-a-one-time-computer-setup)
- [Part B: Flash and bench-test the Nano](#part-b-flash-and-bench-test-the-nano)
- [Part C: Flash the ESP32-S3 and open the live view](#part-c-flash-the-esp32-s3-and-open-the-live-view)
- [Part D: Wiring and power](#part-d-wiring-and-power)
- [Part E: Mount the camera](#part-e-mount-the-camera)
- [Part F: First live tracking](#part-f-first-live-tracking-no-firing)
- [Part G: Calibrate the aim point](#part-g-calibrate-the-aim-point)
- [Part H: Tune](#part-h-tune)
- [Part I: Auto-fire and safety](#part-i-auto-fire-and-safety)
- [Everyday use](#everyday-use)
- [Changing settings and re-flashing](#changing-settings-and-re-flashing)
- [Upgrading from the first version (no live view)](#upgrading-from-the-first-version-no-live-view)
- [Troubleshooting](#troubleshooting)
- [Reference](#reference)
- [Git notes](#git-notes)
- [Credits](#credits)

---

## Repository layout

```
turret-tracker/
├── README.md
├── LICENSE                          MIT (stock turret code is CrunchLabs, MIT)
├── .gitignore                       ignores build output, sdkconfig, wifi_secrets.h
├── nano/
│   └── IRTurret_Tracker/
│       └── IRTurret_Tracker.ino     turret sketch: stock code + serial commands + safety
└── esp32s3_tracker/                 ESP-IDF project for the camera board
    ├── CMakeLists.txt
    ├── sdkconfig.defaults           chip / flash / PSRAM / performance settings
    ├── partitions.csv
    └── main/
        ├── tracker_config.h         ← ALL tunable settings live here
        ├── wifi_secrets.example.h   template; copied to wifi_secrets.h on first build
        ├── app_main.cpp             camera, detector, aiming controller, UART to Nano
        ├── web_view.cpp / .hpp      Wi-Fi, live-view page, MJPEG stream, overlay
        ├── idf_component.yml        esp32-camera, pedestrian_detect, mdns
        └── CMakeLists.txt
```

## Quick reference

| IR remote button | Action |
|---|---|
| Arrows / OK / ★ / 7 / 9 / 0 | Stock behavior (move, fire one, fire all, and so on) |
| **1** | Tracking **on/off**. Off at power-up. Nods yes = on, shakes no = off |
| **#** | Auto-fire **arm/disarm**. Only works while tracking is on. Nods twice = armed |

| Live view | |
|---|---|
| Page | `http://turret.local/`, or `http://<IP>/` using the IP printed in the ESP32 serial log |
| Green box | The person being tracked |
| Yellow box | Other people it sees |
| Cyan dot | Torso point it's steering toward |
| Red crosshair + square | Where darts land (`AIM_X`/`AIM_Y`) and the "close enough" zone. Turns **white** on lock |
| Red border flash | A fire request was sent. The Nano only fires if armed |

---

## Parts

- ESP32-S3-CAM board (N16R8, OV3660, two USB-C ports labeled **OTG** and **TTL**)
- 2 jumper wires, about 30 cm (female Dupont on the ESP32 end)
- Access to the Nano's **RX0 (D0)** and **GND** pins. Either solder short wires to the tops of those pins, or use a spare header if the turret board breaks them out.
- A USB power source for the ESP32: the power bank's second port, or a small separate power bank
- Double-sided foam tape or zip ties
- Safety glasses for anyone downrange during testing

---

## Part A: One-time computer setup

Written for Linux (Ubuntu/Debian). macOS and Windows notes are included where they differ.

### A1. Arduino IDE, for the Nano

1. Install Arduino IDE 2.x.
2. **Boards Manager:** install **Arduino AVR Boards**.
3. **Library Manager:** install **IRremote** by *Armin Joachimsmeyer*, version 4.x. `Servo` is built in.

### A2. ESP-IDF v5.5, for the ESP32

```bash
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv \
     cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.1 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32s3
```

`idf.py` isn't part of this repo. It comes from ESP-IDF and only works after you load ESP-IDF into your terminal. Add a shortcut:

```bash
echo "alias get_idf='. ~/esp/esp-idf/export.sh'" >> ~/.bashrc
source ~/.bashrc
```

From then on, run **`get_idf`** once in every new terminal before using `idf.py`.

- **macOS:** `brew install cmake ninja dfu-util python3`, then the same `git clone` and `install.sh` commands.
- **Windows:** use the ESP-IDF Tools Installer (v5.5.x) or the VS Code *ESP-IDF* extension.

### A3. Serial port permissions (Linux)

```bash
sudo usermod -aG dialout $USER               # then log out and back in
sudo apt remove modemmanager brltty          # both grab USB-serial ports and cause "port busy"
```

---

## Part B: Flash and bench-test the Nano

### B1. Upload the sketch

1. **Disconnect anything wired to the Nano's D0** (the ESP32 link). It blocks uploads.
2. Open `nano/IRTurret_Tracker/IRTurret_Tracker.ino` in Arduino IDE.
3. Set these in the Tools menu:
   - Board: **Arduino AVR Boards → Arduino Nano**. If it still says an ESP32 board from earlier camera testing, the build fails with `SOC_LEDC_TIMER_BIT_WIDE_NUM` errors.
   - Processor: **ATmega328P**. If the upload times out, use **ATmega328P (Old Bootloader)**.
   - Port: the turret's USB port.
4. Click **Upload**.

> You can also paste the whole sketch into the CrunchLabs web IDE, which has IRremote built in.

### B2. Bench test (no camera needed)

1. Open **Serial Monitor**. Set it to **9600 baud** and the line-ending dropdown to **Newline**.
2. Wait for `HOMING`. Opening the monitor resets the Nano, so always open it *before* pressing 1.
3. Press **1** on the remote. The turret nods and prints `TRACKING ON`.
4. Type these commands. Each one is echoed as `CMD …`:

| Type | Expected |
|---|---|
| `P+5` / `P-5` | Tilts up / down 5° |
| `Y+150` | Turns right. If it doesn't move, see B3 |
| `F` | Nothing, because auto-fire isn't armed |
| press **#**, wait 3 s, type `F` | Fires one dart. Press **#** again to disarm |
| press **1**, type `Y+60` | `IGNORED (tracking off, press 1)` |

### B3. Find your turret's yaw numbers

The tracker turns the base with short timed pulses at a reduced speed. Every turret's servo is a bit different, so measure yours. With tracking on:

1. **Speed.** Type `S90` then `Y+150`. That's full speed, the same as the remote arrows, so it should move. Step down with `S70`, `S55`, `S45`, `S40` and so on, sending `Y+100` after each. The lowest value that still moves it reliably is your **tracking speed**.
   - `S` changes are temporary. They reset at power-off.
2. **Minimum pulse.** At that speed, try `Y+15`, `Y+20`, `Y+30`, `Y+40`. The smallest one that visibly moves the base is your **minimum pulse** (ms).
3. **Wire slack.** Send `Y+250` repeatedly and count the commands until the wires to the top start getting tight. Count × 250, minus a margin, is your **yaw soft limit**.

Save the results:

- In the sketch: `int yawTrackSpeed = <tracking speed>;` and `const long YAW_SOFT_LIMIT_MS = <soft limit>;`. Then re-upload (step B1).
- In `esp32s3_tracker/main/tracker_config.h`, which you'll flash in Part C: `#define YAW_MIN_MS <minimum pulse>`.

---

## Part C: Flash the ESP32-S3 and open the live view

### C1. Settings to check before the first flash

Everything is in `esp32s3_tracker/main/tracker_config.h`. For the first flash, only check these:

- `SEND_COMMANDS` is **0**, the shipped default. In this mode it watches and shows, but doesn't move the turret.
- `YAW_MIN_MS` is your number from B3.

### C2. Build

```bash
get_idf
cd ~/path/to/turret-tracker/esp32s3_tracker
idf.py set-target esp32s3        # first time only (and after deleting sdkconfig)
idf.py build
```

The first build downloads the detector model and libraries from Espressif's component registry, so it needs internet and takes 5–10 minutes. It also prints:

```
Created main/wifi_secrets.h from the example. Fill in WIFI_SSID / WIFI_PASSWORD ...
```

### C3. Add your Wi-Fi

Edit **`esp32s3_tracker/main/wifi_secrets.h`**. Don't edit the `.example.h` file.

```c
#define WIFI_SSID     "YourNetworkName"
#define WIFI_PASSWORD "YourPassword"
```

- **2.4 GHz networks only.** The ESP32-S3 can't join 5 GHz-only networks.
- The file is in `.gitignore`, so the password never reaches your repo.

Then run `idf.py build` again.

### C4. Find the ESP32's serial port

Use the board's **TTL** USB-C port for everything. **OTG** isn't needed.

```bash
ls /dev/ttyUSB* /dev/ttyACM*      # run with the ESP32 unplugged, then plugged in
```

The entry that appears is the ESP32. It's usually `/dev/ttyUSB0` or `/dev/ttyACM0`. If the turret's Nano is also plugged in, it has its own port, so don't mix them up.

### C5. Flash and watch the boot log

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

- `monitor` shows the board's log live in the terminal. Press `Ctrl+]` to exit.
- To see the boot log again, press **RST** on the board, or `Ctrl+T` then `Ctrl+R` in the monitor.
- If it hangs at `Connecting....`, hold **BOOT**, tap **RST**, and release **BOOT**.
- `port is busy`: close Arduino's Serial Monitor and any other `monitor` session. `sudo lsof /dev/ttyUSB0` shows what's holding it.

A good boot contains:

```
esp_psram: Found 8MB PSRAM device
tracker: Camera sensor PID 0x3660 (OV3660 = 0x3660)
web: Connecting to Wi-Fi "YourNetworkName"...
web: ==================================================
web:  Live view:  http://turret.local/   or   http://192.168.x.y/
web: ==================================================
tracker: Loading pedestrian detection model...
tracker: Ready. Aim point = (160,120). Commands to Nano: OFF
tracker: 131 ms | 0 det | no target
```

### C6. Open the live view

On any device on the same Wi-Fi, open **`http://turret.local/`**. If that name doesn't resolve (common on Android and some Windows machines), use the IP address from the log.

- **Tip:** give the ESP32 a DHCP reservation in your router so the IP never changes. It appears as `turret`.
- The live view works on power-bank power too, with no computer attached.

### C7. Orientation and detection check

Stand 2–3 m in front of the camera:

1. You should see a **green box** around you and a **cyan dot** on your chest.
2. **Upside-down picture:** flip `CAM_VFLIP` (1 ↔ 0).
3. **Mirrored picture:** hold up a sheet of paper with a word written on it. If the word reads backwards, flip `CAM_HMIRROR`.
4. Rebuild and flash after any change: `idf.py -p /dev/ttyUSB0 flash monitor`.

**No box in good light?** Stand farther back so most of your body is in frame. You can also lower `DETECT_SCORE_THRESHOLD` to 0.5, or try `CAM_RGB565_BIG_ENDIAN 0`; with the wrong value, colors in the live view look wrong too.

---

## Part D: Wiring and power

**Unplug both boards first.**

```
ESP32-S3-CAM                      Turret Nano
  GP14 / GPIO14  ────────────────►  RX0 / D0
  GND            ─────────────────  GND        ← required even with separate power
```

- **No resistor** on the GPIO14 line. The Nano already has one inside.
- **Never** connect the Nano's **TX1** to the ESP32. The Nano sends 5 V, which can damage the ESP32, and nothing needs that direction.
- Use the board's printed label (GP14 / IO14). Don't count pin positions; the header pins aren't in number order.
- If GPIO14 isn't available on your board, pick another free pin listed in `tracker_config.h` and change `NANO_UART_TX_GPIO`.

**Power the ESP32** through its **TTL** USB-C port from the power bank's second port or a separate small bank.

- Only one USB source at a time. When you plug in the computer to watch logs, unplug the power bank.
- Some power banks shut off when the current draw is low. If the ESP32 randomly dies, that's the likely cause.

**Before any Nano upload, unplug the GPIO14 → D0 wire.** A 2-pin inline connector makes this painless.

---

## Part E: Mount the camera

- Mount the ESP32 board on the **pitching barrel assembly**, not the base. The lens should point parallel to the barrel and sit as close to it as practical.
- Keep the orientation you tested in C7. If you mount it differently, recheck C7.
- Route the wires alongside the servo cables with enough slack for full pitch travel and your yaw soft limit.

Mounting it on the barrel means the camera always looks where the gun points, so the tracker only has to move the target onto the crosshair.

---

## Part F: First live tracking (no firing)

1. In `tracker_config.h`, set `#define SEND_COMMANDS 1`. Rebuild and flash.
2. Power everything up. Nothing moves yet, because tracking is off at boot.
3. Open the live view, press **1** on the remote, and walk slowly across the room.
4. **Turns away from you?** Set `YAW_INVERT 1`. **Tilts away?** Set `PITCH_INVERT 1`. Rebuild and flash.
5. Press **1** at any time to stop.

The live view's **last command** readout shows exactly what's being sent: `P+2 Y-35` means tilt up 2° and turn left 35 ms.

---

## Part G: Calibrate the aim point

The camera sits slightly off the barrel, so "where the darts land" isn't the image center.

1. Stand the turret at your usual play distance (about 2.5 m) from a wall or door.
2. With tracking **off**, aim with the remote, fire a few darts, and put tape at the average hit point.
3. Have someone stand with their chest over the tape. **No firing.**
4. On the live view, the cyan dot is on their chest. Read **target point x, y** from the live view (or `tgt=(x,y)` from the serial log) and set `AIM_X` / `AIM_Y` to those numbers. Rebuild and flash.
5. The crosshair should now sit on the tape mark in the live view.

Parallax makes this exact at the distance you calibrated and slightly off nearer or farther, which is fine for foam darts.

---

## Part H: Tune

Change **one** setting at a time in `tracker_config.h`, then rebuild and flash. Watch the live view as you go.

| Symptom | Change |
|---|---|
| Swings past the target and back | Lower `YAW_GAIN_MS_PER_PX` (0.40 → 0.30), or raise `DEADBAND_X_PX` |
| Creeps toward the target too slowly | Raise `YAW_GAIN_MS_PER_PX` (→ 0.55) |
| Small errors never get corrected | Check that `YAW_MIN_MS` is your B3 value |
| Pitch hunts up and down | Lower `PITCH_GAIN_DEG_PER_PX`, or raise `DEADBAND_Y_PX` |
| Overcorrects after moving; picture is blurry | Raise `SETTLE_MS` (120 → 180) |
| Boxes on coats, posters, furniture | Raise `DETECT_SCORE_THRESHOLD` or `MIN_BOX_HEIGHT_PX` |
| Loses a moving target | Raise `REACQUIRE_RADIUS_PX` and `LOST_AFTER_FRAMES` |
| Nano prints `yaw soft limit reached` | Press **1** twice to re-center the limit, or raise `YAW_SOFT_LIMIT_MS` in the sketch if the wires allow |
| Live view too slow or choppy | Lower `WEB_JPEG_QUALITY` (60 → 40). Close extra viewer tabs |

---

## Part I: Auto-fire and safety

1. Tracking on (**1**), then arm (**#**). It nods twice and waits 3 s.
2. It fires when the crosshair has held on the target for 3 frames in a row: `LOCKED`, then `FIRE REQUEST` on the live view.
3. It fires at most one dart per 1.5 s, and **disarms itself after 6 shots**.
4. **#** disarms. **1** turns tracking off and disarms.

**House rules:** safety glasses, foam darts only, and keep `TORSO_FRACTION` at 0.55 or higher so the aim point never moves up toward the head. The arming and shot limits live on the Nano, so the camera board can't fire the turret unless someone pressed **#**.

---

## Everyday use

1. Plug in the turret power bank, then the ESP32 power.
2. Wait about 10 s for the ESP32 to join Wi-Fi. Open `http://turret.local/`.
3. Remote **1** to track. Remote **#** only when everyone downrange has glasses on.

---

## Changing settings and re-flashing

**ESP32:** edit `esp32s3_tracker/main/tracker_config.h`, then:

```bash
get_idf
cd ~/path/to/turret-tracker/esp32s3_tracker
idf.py -p /dev/ttyUSB0 flash monitor      # rebuilds automatically if anything changed
```

**Nano:** edit the `.ino`, unplug the GPIO14 → D0 wire, set Board = Arduino Nano, and upload.

---

## Upgrading from the first version (no live view)

If you built the earlier version from the zip, clear the old build config once, because it doesn't contain the Wi-Fi and PSRAM settings this version needs:

```bash
cd ~/path/to/turret-tracker/esp32s3_tracker
rm -rf build sdkconfig sdkconfig.old managed_components
idf.py set-target esp32s3
idf.py build          # creates main/wifi_secrets.h → fill it in (C3) → build again
```

Carry over any values you had already changed in the old `tracker_config.h` (flip settings, `YAW_MIN_MS`, `AIM_X`/`AIM_Y`, and so on). The new file adds a live-view section at the bottom and now ships with `SEND_COMMANDS 0`.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Arduino: `IRremote.hpp: No such file or directory` | Install **IRremote** by Armin Joachimsmeyer in Library Manager |
| Arduino: `SOC_LEDC_TIMER_BIT_WIDE_NUM` / `esp32/ServoTimers.h` errors | The Board is still set to an ESP32. Set it to **Arduino Nano** |
| Arduino: `avrdude: stk500_recv(): programmer is not responding` | Unplug the D0 wire. Try Processor → **Old Bootloader**. Check the port |
| Serial Monitor commands do nothing | Set it to **9600** and **Newline**. Open the monitor, *then* press **1** (opening the monitor resets the Nano) |
| `CMD Y+…` shows but the base doesn't move | Tracking speed is too low for your servo. Redo B3 |
| `idf.py: command not found` | Run `get_idf` first (Part A2) |
| `Could not open /dev/ttyUSB0 … busy` | Close Arduino Serial Monitor and other monitors. Run `sudo lsof /dev/ttyUSB0`. Remove `modemmanager`/`brltty` |
| `Permission denied` on the port | Run `sudo usermod -aG dialout $USER`, then log out and back in |
| Flash hangs at `Connecting....` | Hold **BOOT**, tap **RST**, release **BOOT** |
| `Camera init failed` | Reseat the ribbon cable and close the latch. If it still fails, your board's pin map differs; update `CAM_PIN_*` from the seller's docs |
| `Live view OFF: set WIFI_SSID …` | Fill in `main/wifi_secrets.h` and rebuild |
| Log repeats `Wi-Fi disconnected (reason …)` | Wrong password, or a 5 GHz-only network. The ESP32 needs 2.4 GHz |
| `turret.local` doesn't open | Use the IP from the log. Android and some Windows setups don't resolve `.local` names |
| Page loads but the picture is blank | Port 81 must be reachable. Some guest or isolated Wi-Fi networks block device-to-device traffic |
| Turret ignores the camera | Tracking is off (press **1**), `SEND_COMMANDS 0`, GND not connected, wrong GPIO, or the D0 wire is loose |
| Twitches with nobody in view | A false detection; the live view shows what it's seeing. Raise `DETECT_SCORE_THRESHOLD` |
| ESP32 resets when the servos move | Power it from its own USB source (Part D) |
| About 250 ms per frame or more | `sdkconfig` is stale. Delete `build/` and `sdkconfig`, then `set-target` and build again |

---

## Reference

### ESP32 → Nano protocol (9600 8N1, newline-terminated)

| Command | Meaning | Nano-side limit |
|---|---|---|
| `Y+n` / `Y-n` | Yaw right/left for *n* ms at `yawTrackSpeed` | ±250 ms per command; ±`YAW_SOFT_LIMIT_MS` total |
| `P+n` / `P-n` | Pitch up/down *n* degrees | ±15° per command; `pitchMin`..`pitchMax` |
| `F` | Fire one dart | Only when armed; 3 s grace, 1.5 s cooldown, 6 shots |
| `Sn` (e.g. `S45`) | Set `yawTrackSpeed` live (10–90), for tuning | Accepted even with tracking off; not saved |

`Y`, `P` and `F` are ignored unless tracking is on (remote **1**). Every command is echoed in Serial Monitor.

### Web endpoints (ESP32)

| URL | Returns |
|---|---|
| `http://turret.local/` | Live-view page |
| `http://turret.local/status` | JSON with frame time, detections, target, error, last command, lock/fire state |
| `http://turret.local:81/stream` | MJPEG stream with the overlay drawn in, usable in VLC or other tools |

### `tracker_config.h` sections

| Section | Key settings |
|---|---|
| Link to Nano | `NANO_UART_TX_GPIO`, `NANO_UART_BAUD` |
| Camera | `CAM_PIN_*`, `CAM_VFLIP`, `CAM_HMIRROR`, `CAM_RGB565_BIG_ENDIAN` |
| Detection | `DETECT_SCORE_THRESHOLD`, `MIN_BOX_HEIGHT_PX`, `TORSO_FRACTION`, `REACQUIRE_RADIUS_PX`, `LOST_AFTER_FRAMES` |
| Aim point | `AIM_X`, `AIM_Y` |
| Control loop | `DEADBAND_*`, `YAW_GAIN_MS_PER_PX`, `YAW_MIN_MS`, `YAW_MAX_MS`, `YAW_INVERT`, `PITCH_GAIN_DEG_PER_PX`, `PITCH_MAX_DEG`, `PITCH_INVERT`, `SETTLE_MS`, `LOCK_FRAMES_TO_FIRE`, `SEND_COMMANDS` |
| Live view | `ENABLE_WEB_VIEW`, `WEB_HOSTNAME`, `WEB_JPEG_QUALITY` |

---

## Git notes

- **Ignored:** `build/`, `managed_components/`, `sdkconfig`, `sdkconfig.old`, and `main/wifi_secrets.h`. Your Wi-Fi password stays local.
- **Commit after your first successful build:** `esp32s3_tracker/dependencies.lock`. It pins the exact versions of the detector, camera and mDNS components, so future builds are reproducible.
- **Fresh clone on another machine:** `get_idf`, `cd esp32s3_tracker`, `idf.py set-target esp32s3`, `idf.py build`, fill in `wifi_secrets.h`, then build again.

To start the repo:

```bash
cd ~/path/to/turret-tracker
git init
git add .
git status        # confirm wifi_secrets.h, build/ and sdkconfig are NOT listed
git commit -m "Turret tracker: ESP32-S3 vision + live view, Nano serial control"
```

---

## Credits

- Stock turret code: © CrunchLabs, MIT license, via [billism1/hackpack-irturret-customization](https://github.com/billism1/hackpack-irturret-customization).
- Serial-control idea: [martyn-johnson/ir_turret](https://github.com/martyn-johnson/ir_turret) (Raspberry Pi version).
- Detector: [espressif/esp-dl](https://github.com/espressif/esp-dl), `pedestrian_detect` model (PicoDet, 224×224).
- Camera driver and JPEG encoder: [espressif/esp32-camera](https://github.com/espressif/esp32-camera).
- Firing-mechanism tuning: [elearningplugins/crunchlabs-ir-turret-fix](https://github.com/elearningplugins/crunchlabs-ir-turret-fix).
- Next step for tracking a specific object instead of people: [espressif/esp-detection](https://github.com/espressif/esp-detection), which trains a one-class, YOLO-based `.espdl` model.
