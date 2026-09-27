/*
************************************************************************************
* MIT License
*
* Copyright (c) 2023 Crunchlabs LLC (IRTurret Control Code)
* Copyright (c) 2020-2022 Armin Joachimsmeyer (IRremote Library)
*
* Tracking additions (serial command interface, tracking / auto-fire modes,
* safety limits) added on top of the stock CrunchLabs IR Turret sketch.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is furnished
* to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
* CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
* OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
************************************************************************************
*
* ============================ WHAT THIS VERSION ADDS ============================
*
*  The ESP32-S3 camera board sends short text commands to this Nano over the
*  Nano's hardware serial RX pin (D0 / "RX0"), 9600 baud, one command per line:
*
*     Y+40   yaw RIGHT for 40 ms      Y-40   yaw LEFT for 40 ms   (max 250 ms)
*     P+3    pitch UP 3 degrees       P-3    pitch DOWN 3 degrees (max 15)
*     F      request one auto-shot (only honored when auto-fire is ARMED)
*     S45    set yawTrackSpeed to 45 on the fly (tuning helper, 10..90, not saved)
*     K20    set the yaw kick-start to 20 ms on the fly (tuning helper, 0..60, not saved)
*
*  Every command is echoed in Serial Monitor ("CMD Y+60", or "IGNORED ..."
*  if tracking is off), so you can see whether it arrived.
*
*  Everything from the stock sketch still works on the IR remote. New buttons:
*
*     1   TRACKING on/off   (default OFF at power-up; turret nods yes = on,
*                            shakes no = off). Serial commands are ignored
*                            while tracking is off.
*     #   AUTO-FIRE arm/disarm (only works while tracking is on; default OFF).
*         Arming nods yes twice, disarming shakes no once. After arming there
*         is a 3-second grace period, then at most one shot every 1.5 s, and it
*         disarms itself after 6 shots (one full barrel).
*
*  You can bench-test without the camera: open Serial Monitor at 9600 baud
*  ("Newline" line ending), press 1 on the remote, then type Y+60 or P+5.
*
*  IMPORTANT: unplug the ESP32 -> D0 wire before uploading this sketch over USB.
*  The ESP32 drives that pin and will make the upload fail.
* ================================================================================
*/

//////////////////////////////////////////////////
              //  LIBRARIES  //
//////////////////////////////////////////////////
#include <Arduino.h>
#include <Servo.h>
#include <IRremote.hpp>


#define DECODE_NEC  //defines the type of IR transmission to decode based on the remote.

//defines the specific command code for each button on the remote
#define left 0x8
#define right 0x5A
#define up 0x52
#define down 0x18
#define ok 0x1C
#define cmd1 0x45
#define cmd2 0x46
#define cmd3 0x47
#define cmd4 0x44
#define cmd5 0x40
#define cmd6 0x43
#define cmd7 0x7
#define cmd8 0x15
#define cmd9 0x9
#define cmd0 0x19
#define star 0x16
#define hashtag 0xD

//////////////////////////////////////////////////
          //  PINS AND PARAMETERS  //
//////////////////////////////////////////////////
Servo yawServo;   // YAW rotation, 360 spin around the base (continuous-rotation servo)
Servo pitchServo; // PITCH rotation, up and down tilt (positional servo)
Servo rollServo;  // ROLL rotation, spins the barrel to fire darts (continuous-rotation servo)

int yawServoVal;
int pitchServoVal = 100;
int rollServoVal;

int pitchMoveSpeed = 8;   // degrees per manual up/down press
int yawMoveSpeed = 90;    // manual (remote) yaw speed, added/subtracted from yawStopSpeed
int yawStopSpeed = 90;    // value that stops the yaw motor - keep at 90
int rollMoveSpeed = 90;   // keep at 90 for max firing torque
int rollStopSpeed = 90;   // value that stops the roll motor - keep at 90

int yawPrecision = 150;   // ms per manual yaw press
int rollPrecision = 158;  // ms of barrel spin per dart (~1/6 turn). Tune 150-170 if darts misfeed.

int pitchMax = 175;       // pitch limits - keep inside 0..180 so the servo never crashes
int pitchMin = 10;

// Stock turrets tilt UP as the pitch servo value goes DOWN. Set true if yours tilts the other
// way (remote down arrow raises the barrel, "P+5" lowers it). Fixes the remote and the camera.
const bool PITCH_REVERSED = false;
const int  PITCH_UP_STEP = PITCH_REVERSED ? 1 : -1;  // servo value change per degree of "up"

//////////////////////////////////////////////////
       //  TRACKING PARAMETERS (NEW)  //
//////////////////////////////////////////////////
int yawTrackSpeed = 40;                     // slower than manual (90) so small corrections are precise.
                                            // Find the right value for YOUR turret with the S command
                                            // in Serial Monitor (see README, Part B), then put it here.
int yawKickMs = 50;                         // each tracking yaw pulse starts with this many ms at full
                                            // speed to break the base free of static friction, then
                                            // drops to yawTrackSpeed. 0 = no kick. Tune with K.
const int  YAW_PULSE_MAX_MS = 250;          // longest single yaw pulse accepted from the camera
const int  PITCH_STEP_MAX = 15;             // biggest single pitch step accepted from the camera
const long YAW_SOFT_LIMIT_MS = 1800;        // net yaw travel allowed either side of where tracking was
                                            // switched on (in ms at yawTrackSpeed). Stops the turret
                                            // winding its wires around the base. Tune after testing.
const unsigned long AUTO_FIRE_COOLDOWN_MS = 1500; // minimum time between auto shots
const unsigned long ARM_GRACE_MS = 3000;          // no auto shots for this long after arming
const int  AUTO_SHOTS_PER_ARM = 6;                // disarm after one full barrel

bool trackingEnabled = false;
bool autoFireArmed = false;
int  autoShotsLeft = 0;
unsigned long armedAtMs = 0;
unsigned long lastAutoShotMs = 0;
long yawOdometerMs = 0;                     // + = net right, - = net left, since tracking enabled
const unsigned long TOGGLE_LOCKOUT_MS = 600; // ignore 1 / # for this long after a toggle finishes
unsigned long lastToggleMs = 0;

char cmdBuf[16];
uint8_t cmdLen = 0;
bool discardingLine = false;

void shootAroundRandomly();
void shakeHeadYes(int moves);
void shakeHeadNo(int moves);
void leftMove(int moves);
void rightMove(int moves);
void upMove(int moves);
void downMove (int moves);
void fire();
void fireAll();
void homeServos();

void pollSerialCommands();
void handleSerialCommand(const char *cmd);
void trackYaw(int ms);
void trackPitch(int deg);
void autoFire();
void toggleTracking();
void toggleAutoFire();
void flushSerialInput();
bool toggleKeyAccepted();

//////////////////////////////////////////////////
              //  S E T U P  //
//////////////////////////////////////////////////
void setup() {
    Serial.begin(9600); // USB serial AND the ESP32 link (ESP32 TX -> Nano RX0/D0)

    yawServo.attach(10);   //attach YAW servo to pin 10
    pitchServo.attach(11); //attach PITCH servo to pin 11
    rollServo.attach(12);  //attach ROLL servo to pin 12

    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

    IrReceiver.begin(9, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
    Serial.println("at pin 9");
    Serial.println(F("Tracking build: press 1 = tracking on/off, # = auto-fire arm/disarm"));

    homeServos(); //set servo motors to home position
}

////////////////////////////////////////////////
              //  L O O P  //
////////////////////////////////////////////////

void loop() {

    // NEW: handle commands from the ESP32 camera board (or typed in Serial Monitor)
    pollSerialCommands();

    if (IrReceiver.decode()) {

        IrReceiver.printIRResultShort(&Serial);
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) { //command garbled or not recognized
            Serial.println(F("Received noise or an unknown protocol"));
        }
        Serial.println();

        IrReceiver.resume(); // Enable receiving of the next value

        switch(IrReceiver.decodedIRData.command){ //this is where the commands are handled

            case up://pitch up
              upMove(1);
              break;

            case down://pitch down
              downMove(1);
              break;

            case left://fast counterclockwise rotation
              leftMove(1);
              break;

            case right://fast clockwise rotation
              rightMove(1);
              break;

            case ok: //firing routine
              fire();
              break;

            case star:
              fireAll();
              delay(50);
              break;

            case cmd0:
              shakeHeadNo(3);
              delay(50);
              break;

            case cmd9:
              shakeHeadYes(3);
              delay(50);
              break;

            case cmd7:
              shootAroundRandomly();
              break;

            case cmd1: // NEW: tracking on/off
              if (toggleKeyAccepted()) {
                toggleTracking();
                lastToggleMs = millis();
              }
              break;

            case hashtag: // NEW: auto-fire arm/disarm
              if (toggleKeyAccepted()) {
                toggleAutoFire();
                lastToggleMs = millis();
              }
              break;
        }
    }
    delay(5);
}

////////////////////////////////////////////////
      //  NEW: SERIAL / TRACKING CODE  //
////////////////////////////////////////////////

// Reads characters without blocking; runs a command each time a full line arrives.
void pollSerialCommands() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (!discardingLine && cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                handleSerialCommand(cmdBuf);
            }
            cmdLen = 0;
            discardingLine = false;
        } else if (discardingLine) {
            // skip the rest of an over-long / garbage line
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        } else {
            cmdLen = 0;
            discardingLine = true;
        }
    }
}

void handleSerialCommand(const char *cmd) {
    char op = cmd[0];
    int val = atoi(cmd + 1); // accepts "+40", "-40", "40"

    // Tuning helper: "S45" sets yawTrackSpeed live (10..90), no re-upload needed.
    // Works whether or not tracking is on. Not saved: resets on power-up.
    if (op == 'S') {
        yawTrackSpeed = constrain(val, 10, 90);
        Serial.print(F("yawTrackSpeed = "));
        Serial.println(yawTrackSpeed);
        return;
    }

    // Tuning helper: "K20" sets yawKickMs live (0..60). Not saved: resets on power-up.
    if (op == 'K') {
        yawKickMs = constrain(val, 0, 60);
        Serial.print(F("yawKickMs = "));
        Serial.println(yawKickMs);
        return;
    }

    if (!trackingEnabled) {
        Serial.print(F("IGNORED (tracking off, press 1): "));
        Serial.println(cmd);
        return; // camera commands do nothing until you press 1 on the remote
    }
    Serial.print(F("CMD "));
    Serial.println(cmd);
    switch (op) {
        case 'Y': trackYaw(val);   break;
        case 'P': trackPitch(val); break;
        case 'F': autoFire();      break;
        default:  break;          // ignore anything else
    }
}

// Positive ms = turn RIGHT (same direction as the remote's right button).
void trackYaw(int ms) {
    ms = constrain(ms, -YAW_PULSE_MAX_MS, YAW_PULSE_MAX_MS);
    if (ms == 0) return;

    long next = yawOdometerMs + ms;
    if (next > YAW_SOFT_LIMIT_MS || next < -YAW_SOFT_LIMIT_MS) {
        Serial.println(F("TRACK: yaw soft limit reached"));
        return;
    }

    // Right (clockwise) is below yawStopSpeed and left is above, matching rightMove() / leftMove().
    int dir = (ms > 0) ? -1 : 1;
    int total = abs(ms);
    int kick = min(yawKickMs, total);
    if (kick > 0) {
        yawServo.write(yawStopSpeed + dir * yawMoveSpeed); // full-speed burst to get it moving
        delay(kick);
    }
    if (total > kick) {
        yawServo.write(yawStopSpeed + dir * yawTrackSpeed);
        delay(total - kick);
    }
    yawServo.write(yawStopSpeed);
    yawOdometerMs = next;
}

// Positive deg = aim UP. PITCH_REVERSED sets which way that turns the servo (see upMove()).
void trackPitch(int deg) {
    deg = constrain(deg, -PITCH_STEP_MAX, PITCH_STEP_MAX);
    if (deg == 0) return;
    pitchServoVal = constrain(pitchServoVal + PITCH_UP_STEP * deg, pitchMin, pitchMax);
    pitchServo.write(pitchServoVal);
}

void autoFire() {
    if (!autoFireArmed) return;
    unsigned long now = millis();
    if (now - armedAtMs < ARM_GRACE_MS) return;
    if (lastAutoShotMs != 0 && now - lastAutoShotMs < AUTO_FIRE_COOLDOWN_MS) return;
    if (autoShotsLeft <= 0) return;

    fire();
    lastAutoShotMs = millis();
    autoShotsLeft--;
    Serial.print(F("AUTO-FIRE: shots left "));
    Serial.println(autoShotsLeft);

    if (autoShotsLeft <= 0) {
        autoFireArmed = false;
        Serial.println(F("AUTO-FIRE: barrel empty, disarmed"));
        shakeHeadNo(1);
    }
}

// On/off keys must act once per press. The remote sends "repeat" frames while a key is held
// (and sometimes a second full frame right after the first), which would toggle straight back.
bool toggleKeyAccepted() {
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) return false;
    if (millis() - lastToggleMs < TOGGLE_LOCKOUT_MS) return false;
    return true;
}

void toggleTracking() {
    trackingEnabled = !trackingEnabled;
    autoFireArmed = false; // changing tracking state always disarms
    if (trackingEnabled) {
        yawOdometerMs = 0; // wherever it is pointing now becomes "center" for the soft limit
        shakeHeadYes(1);
        flushSerialInput();  // drop anything the camera sent during the nod
        Serial.println(F("TRACKING ON"));
    } else {
        shakeHeadNo(1);
        flushSerialInput();
        Serial.println(F("TRACKING OFF"));
    }
}

void toggleAutoFire() {
    if (!trackingEnabled) {
        shakeHeadNo(1); // refuse: turn tracking on first
        flushSerialInput();
        Serial.println(F("AUTO-FIRE: turn tracking on first (button 1)"));
        return;
    }
    autoFireArmed = !autoFireArmed;
    if (autoFireArmed) {
        autoShotsLeft = AUTO_SHOTS_PER_ARM;
        lastAutoShotMs = 0;
        shakeHeadYes(2);
        armedAtMs = millis(); // grace period starts after the nod finishes
        Serial.println(F("AUTO-FIRE ARMED"));
    } else {
        shakeHeadNo(1);
        Serial.println(F("AUTO-FIRE DISARMED"));
    }
    flushSerialInput();
}

void flushSerialInput() {
    while (Serial.available() > 0) {
        Serial.read();
    }
    cmdLen = 0;
    discardingLine = false;
}

////////////////////////////////////////////////
      //  STOCK MOVEMENT FUNCTIONS  //
////////////////////////////////////////////////

void shootAroundRandomly() {
  rollServo.write(rollStopSpeed + rollMoveSpeed);//start rotating the servo
  leftMove(6);
  rollServo.write(rollStopSpeed);//stop rotating the servo
  delay(5); // delay for smoothness
  Serial.println("FIRING ALL RANDOMLY AROUND");
}

void shakeHeadYes(int moves = 3) {
    Serial.println("YES");
    int startAngle = pitchServoVal; // Current position of the pitch servo
    int nodAngle = startAngle + 20; // Angle for nodding motion

    for (int i = 0; i < moves; i++) {
        for (int angle = startAngle; angle <= nodAngle; angle++) {
            pitchServo.write(angle);
            delay(7);
        }
        delay(50);
        for (int angle = nodAngle; angle >= startAngle; angle--) {
            pitchServo.write(angle);
            delay(7);
        }
        delay(50);
    }
}

void shakeHeadNo(int moves = 3) {
    Serial.println("NO");
    for (int i = 0; i < moves; i++) {
        yawServo.write(140);
        delay(190);
        yawServo.write(yawStopSpeed);
        delay(50);
        yawServo.write(40);
        delay(190);
        yawServo.write(yawStopSpeed);
        delay(50);
    }
}

void leftMove(int moves){
    for (int i = 0; i < moves; i++){
        yawServo.write(yawStopSpeed + yawMoveSpeed); // full counterclockwise speed
        delay(yawPrecision);
        yawServo.write(yawStopSpeed);
        delay(5);
        Serial.println("LEFT");
  }
}

void rightMove(int moves){
  for (int i = 0; i < moves; i++){
      yawServo.write(yawStopSpeed - yawMoveSpeed); // full clockwise speed
      delay(yawPrecision);
      yawServo.write(yawStopSpeed);
      delay(5);
      Serial.println("RIGHT");
  }
}

void upMove(int moves){
  for (int i = 0; i < moves; i++){
      int next = pitchServoVal + PITCH_UP_STEP * pitchMoveSpeed;
      if(next >= pitchMin && next <= pitchMax){
        pitchServoVal = next;
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("UP");
      }
  }
}

void downMove (int moves){
  for (int i = 0; i < moves; i++){
        int next = pitchServoVal - PITCH_UP_STEP * pitchMoveSpeed;
        if(next >= pitchMin && next <= pitchMax){
        pitchServoVal = next;
        pitchServo.write(pitchServoVal);
        delay(50);
        Serial.println("DOWN");
      }
  }
}

void fire() { //function for firing a single dart
    rollServo.write(rollStopSpeed + rollMoveSpeed);
    delay(rollPrecision);
    rollServo.write(rollStopSpeed);
    delay(5);
    Serial.println("FIRING");
}

void fireAll() { //function to fire all 6 darts at once
    rollServo.write(rollStopSpeed + rollMoveSpeed);
    delay(rollPrecision * 6);
    rollServo.write(rollStopSpeed);
    delay(5);
    Serial.println("FIRING ALL");
}

void homeServos(){
    yawServo.write(yawStopSpeed);
    delay(20);
    rollServo.write(rollStopSpeed);
    delay(100);
    pitchServo.write(100);
    delay(100);
    pitchServoVal = 100;
    Serial.println("HOMING");
}
