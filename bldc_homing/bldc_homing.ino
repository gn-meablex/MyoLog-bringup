#include <SimpleFOC.h>
#include <Wire.h>
#include "MT6701.h"

// ── Motor & driver ──────────────────────────────────────────────
BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(21, 18, 14);

// ── Hall sensor — linked to SimpleFOC for FOC alignment/commutation ──
HallSensor hall = HallSensor(3, 12, 46, 1);
void doA(){ hall.handleA(); }
void doB(){ hall.handleB(); }
void doC(){ hall.handleC(); }

// ── MT6701 — read directly, NOT linked to SimpleFOC ──────────────
MT6701 encoder;
float lastRawAngle = 0;
float unwrappedAngle = 0;
bool mtInitialized = false;

// Call every loop to keep a continuous (unwrapped) MT6701 angle in degrees
float updateMT6701() {
  float raw = encoder.angleRead();  // 0–360 deg
  if (!mtInitialized) {
    lastRawAngle = raw;
    unwrappedAngle = raw;
    mtInitialized = true;
  }
  float delta = raw - lastRawAngle;
  if (delta > 180.0) delta -= 360.0;
  else if (delta < -180.0) delta += 360.0;
  unwrappedAngle += delta;
  lastRawAngle = raw;
  return unwrappedAngle;
}

// ── Travel constant ───────────────────────────────────────────────
const float TRAVEL_DEG = 1.0f * 360.0f;  // tested/known, 1.5 rev

// ── Calibrated reference (degrees, MT6701 frame) ─────────────────
float homeDeg = 0;
float closeTargetDeg = 0;
float openTargetDeg = 0;
bool calibrated = false;

// ── Jog settings ───────────────────────────────────────────────────
const float JOG_STEP_DEG = 30.0f;
const float JOG_VOLTAGE = 3.0f;   // keep low/gentle for manual jogging — tune as needed
float jogTargetDeg = 0;

// ── Motion voltage for close/open/return ─────────────────────────
const float MOVE_VOLTAGE = 6.0f;  // tune as needed — start lower with wire tensioned

// ── State ─────────────────────────────────────────────────────────
enum HandState { IDLE, CLOSING, OPENING, RETURNING_HOME, JOGGING };
HandState state = IDLE;

// ── Stall detection — based on MT6701 position deltas ────────────
const float STALL_DELTA_DEG_THRESH = 0.5f;  // deg moved per check window
const uint32_t STALL_CHECK_MS = 100;
uint32_t lastStallCheck = 0;
float posAtLastStallCheck = 0;
bool stallInit = false;

bool checkStalled(float currentDeg) {
  uint32_t now = millis();
  if (!stallInit) {
    posAtLastStallCheck = currentDeg;
    lastStallCheck = now;
    stallInit = true;
    return false;
  }
  if (now - lastStallCheck >= STALL_CHECK_MS) {
    float moved = fabs(currentDeg - posAtLastStallCheck);
    posAtLastStallCheck = currentDeg;
    lastStallCheck = now;
    return moved < STALL_DELTA_DEG_THRESH;
  }
  return false;
}

// ── Serial commands ───────────────────────────────────────────────
Commander command = Commander(Serial);
void doCalibrate(char* cmd) {
  // Zero the encoder here: reset both raw and unwrapped tracking
  lastRawAngle = encoder.angleRead();
  unwrappedAngle = 0;

  homeDeg = 0;
  closeTargetDeg = homeDeg - TRAVEL_DEG;  // reversed — was + TRAVEL_DEG
  openTargetDeg  = homeDeg;
  calibrated = true;
  Serial.print("Calibrated. Home (deg): ");
  Serial.println(homeDeg);
}

void doStop(char* cmd) {
  motor.move(0);
  state = IDLE;
  stallInit = false;
  Serial.println("STOPPED.");
}
void doClose(char* cmd) {
  if (!calibrated) { Serial.println("Not calibrated — send H first."); return; }
  if (state == IDLE) { stallInit = false; state = CLOSING; }
}

void doOpen(char* cmd) {
  if (!calibrated) { Serial.println("Not calibrated — send H first."); return; }
  if (state == IDLE) { stallInit = false; state = OPENING; }
}

void doReturnHome(char* cmd) {
  if (!calibrated) { Serial.println("Not calibrated — send H first."); return; }
  state = RETURNING_HOME;
  stallInit = false;
  Serial.println("Returning to home...");
}

void doJogForward(char* cmd) {
  if (state != IDLE) { Serial.println("Busy — wait for current action to finish."); return; }
  jogTargetDeg = unwrappedAngle + JOG_STEP_DEG;  // flip sign if this winds the wrong way
  state = JOGGING;
  stallInit = false;
  Serial.print("Jogging forward to (deg): ");
  Serial.println(jogTargetDeg);
}

void doJogReverse(char* cmd) {
  if (state != IDLE) { Serial.println("Busy — wait for current action to finish."); return; }
  jogTargetDeg = unwrappedAngle - JOG_STEP_DEG;
  state = JOGGING;
  stallInit = false;
  Serial.print("Jogging reverse to (deg): ");
  Serial.println(jogTargetDeg);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(10, 9, 100000);
  encoder.initializeI2C();

  pinMode(17, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(17, HIGH);

  hall.init();
  hall.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&hall);

  driver.voltage_power_supply = 12;  // deliberate attenuator — do not change
  driver.init();
  motor.linkDriver(&driver);

  motor.voltage_sensor_align = 3;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();  // aligns against Hall — proven working

  command.add('H', doCalibrate, "set current position as home (open)");
  command.add('C', doClose, "close hand");
  command.add('O', doOpen, "open hand");
  command.add('R', doReturnHome, "return to home");
  command.add('F', doJogForward, "jog forward 10 deg");
  command.add('B', doJogReverse, "jog reverse 10 deg");
  command.add('S', doStop, "STOP motor immediately");

  Serial.println(F("Ready. Position hand fully open, send H to calibrate."));
  Serial.println(F("Commands: C=close O=open R=return-home F=jog-fwd-10deg B=jog-rev-10deg S=STOP"));
}

void loop() {
  motor.loopFOC();
  command.run();

  float posDeg = updateMT6701();  // MT6701, read independently

  switch (state) {

    case CLOSING: {
      bool pastTarget = (closeTargetDeg > homeDeg) ? (posDeg >= closeTargetDeg) : (posDeg <= closeTargetDeg);
      if (pastTarget) {
        motor.move(0);
        Serial.println("Close complete.");
        state = IDLE;
      } else if (checkStalled(posDeg)) {
        motor.move(0);
        float percentClosed = fabs(posDeg - homeDeg) / TRAVEL_DEG * 100.0f;
        Serial.print("Stalled while closing at "); Serial.print(percentClosed); Serial.println("% closed");
        state = IDLE;
      } else {
        motor.move((closeTargetDeg > homeDeg) ? MOVE_VOLTAGE : -MOVE_VOLTAGE);
      }
      break;
    }

    case OPENING: {
      if (fabs(posDeg - openTargetDeg) < 2.0f) {
        motor.move(0);
        Serial.println("Open complete.");
        state = IDLE;
      } else if (checkStalled(posDeg)) {
        motor.move(0);
        Serial.println("Stalled while opening.");
        state = IDLE;
      } else {
        motor.move((openTargetDeg < posDeg) ? -MOVE_VOLTAGE : MOVE_VOLTAGE);
      }
      break;
    }

    case RETURNING_HOME: {
      if (fabs(posDeg - homeDeg) < 2.0f) {
        motor.move(0);
        Serial.println("Returned home.");
        state = IDLE;
      } else if (checkStalled(posDeg)) {
        motor.move(0);
        Serial.print("Blocked on return — stopped at (deg): ");
        Serial.println(posDeg);
        state = IDLE;
      } else {
        motor.move((homeDeg < posDeg) ? -MOVE_VOLTAGE : MOVE_VOLTAGE);
      }
      break;
    }

    case JOGGING: {
      if (fabs(posDeg - jogTargetDeg) < 0.5f) {
        motor.move(0);
        Serial.print("Jog complete. Pos (deg): ");
        Serial.println(posDeg);
        state = IDLE;
      } else if (checkStalled(posDeg)) {
        motor.move(0);
        Serial.print("Jog stalled at (deg): ");
        Serial.println(posDeg);
        state = IDLE;
      } else {
        motor.move((jogTargetDeg > posDeg) ? JOG_VOLTAGE : -JOG_VOLTAGE);
      }
      break;
    }

    case IDLE:
      motor.move(0);
      break;
  }

  static uint32_t taskTick = 0;
  if (millis() - taskTick >= 100) {
    taskTick = millis();
    Serial.print("MT Pos(deg): "); Serial.print(posDeg);
    Serial.print("  Calibrated: "); Serial.print(calibrated);
    Serial.print("  State: "); Serial.println(state);
  }
}