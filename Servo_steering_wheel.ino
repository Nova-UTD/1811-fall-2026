// 1811 vehicle firmware - Arduino Uno
// Input (newline-terminated JSON at 57600 baud):
// {"speed": 0.0, "steering": 0.0, "braking": 0.0}
// steering: -1 (left) ... 0 (center) ... +1 (right)
// Outputs:
// Docyke S350 axle steering: ServoTimer2, pin 10
// Brake servo: ServoTimer2, pin 6
// Hitec HS-422 steering-wheel servo: ServoTimer2, pin 3
// VESC: AltSoftSerial RX 8, TX 9 (19200 baud)

#include <Arduino.h>
#include <AltSoftSerial.h>
#include <VescUart.h>
#include <ServoTimer2.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

AltSoftSerial vescSerial;
VescUart VESC;
ServoTimer2 steer;
ServoTimer2 brake;
ServoTimer2 steeringWheel;  // Correct capitalization everywhere

const uint8_t STEERING_WHEEL_PIN = 3;
const int WHEEL_SERVO_MIN_US = 900;     // Nominal HS-422 safe operating range
const int WHEEL_SERVO_CENTER_US = 1500;
const int WHEEL_SERVO_MAX_US = 2100;

#define BAUD_RATE 57600
#define MSG_BUF_SIZE 96
#define STALE_TIMEOUT_MS 250UL
#define SERVO_UPDATE_MS 20UL
#define VESC_TX_INTERVAL_MS 20UL

// S350 axle calibration from the original vehicle firmware: -25/0/+25 degrees.
const float STEER_CENTER_US = 1150.0f;
const float STEER_SPAN_US = 416.667f;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Steering pipeline - shared smooth input for both steering servos.
const float STEER_SLEW_PER_UPDATE = 0.08f;
const float STEER_DEADBAND = 0.02f;
const float BACKLASH_MOVING_POS = 0.08f; // S350 only; calibrated 2 degrees
const float BACKLASH_MOVING_NEG = 0.00f;
const float DIRECTION_EPS = 0.001f;

const float WHEEL_DIAMETER_IN = 12.0f;
const float GEAR_RATIO = 1.0f;
const int MOTOR_POLE_PAIRS = 7;

char msgBuf[MSG_BUF_SIZE];
uint8_t msgLen = 0;
float speed = 0.0f;
float steering = 0.0f;
float braking = 0.0f;
float steerApplied = 0.0f;
int steerDirection = 0;
unsigned long lastServoUpdateMs = 0;
unsigned long lastVescTxMs = 0;
unsigned long lastValidMsgMs = 0;
bool linkStale = true;

float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool parseField(const char *buf, const char *key, float *out) {
  const char *found = strstr(buf, key);
  if (!found) return false;
  const char *start = found + strlen(key);
  char *end;
  float val = strtod(start, &end);
  if (end == start || !isfinite(val)) return false;
  *out = val;
  return true;
}

// Process complete JSON lines without blocking while waiting for a newline.
void readAndParseSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      msgBuf[msgLen] = '\0';
      float s, st, b;
      if (parseField(msgBuf, "\"speed\":", &s) &&
          parseField(msgBuf, "\"steering\":", &st) &&
          parseField(msgBuf, "\"braking\":", &b)) {
        speed = s;
        steering = clampFloat(st, -1.0f, 1.0f);
        braking = clampFloat(b, 0.0f, 1.0f);
        lastValidMsgMs = millis();
        linkStale = false;
      }
      msgLen = 0;
    } else if (ch != '\r') {
      if (msgLen < MSG_BUF_SIZE - 1) {
        msgBuf[msgLen++] = ch;
      } else {
        // Discard an oversized malformed line, rather than keep a truncated one.
        msgLen = 0;
      }
    }
  }
}

void checkStaleness() {
  if (!linkStale && millis() - lastValidMsgMs > STALE_TIMEOUT_MS) {
    linkStale = true;
    speed = 0.0f;
    steering = 0.0f;
    braking = 0.0f; // Preserve original coasting policy: NOT active braking.
    // steerApplied will slew smoothly back to center.
  }
}

// Physical axle steering, with direction-dependent backlash compensation.
void writeSteeringNormalized(float s) {
  float us = STEER_CENTER_US + STEER_SPAN_US * s;
  us = clampFloat(us, SERVO_MIN_US, SERVO_MAX_US);
  steer.write((int)(us + 0.5f));
}

// Hitec HS-422 follows the same slew-limited steering value, without
// borrowing the S350-specific backlash correction.
void writeSteeringWheelNormalized(float s) {
  s = clampFloat(s, -1.0f, 1.0f);
  float us;
  if (s < 0.0f) {
    us = WHEEL_SERVO_CENTER_US + s * (WHEEL_SERVO_CENTER_US - WHEEL_SERVO_MIN_US);
  } else {
    us = WHEEL_SERVO_CENTER_US + s * (WHEEL_SERVO_MAX_US - WHEEL_SERVO_CENTER_US);
  }
  steeringWheel.write((int)(us + 0.5f));
}

int convertToMicro(float angle) {
  return (int)(500.0f + (angle / 360.0f) * 2000.0f);
}

void applyBrake(float amount) {
  float actAngle = 300.0f - (70.0f * amount);
  brake.write(convertToMicro(actAngle));
}

void updateActuators() {
  unsigned long now = millis();
  if (now - lastServoUpdateMs < SERVO_UPDATE_MS) return;
  lastServoUpdateMs = now;

  float target = clampFloat(steering, -1.0f, 1.0f);
  // Center small joystick jitter at zero; otherwise ignore tiny changes.
  if (fabs(target) < STEER_DEADBAND) target = 0.0f;
  else if (fabs(target - steerApplied) < STEER_DEADBAND) target = steerApplied;

  float delta = clampFloat(target - steerApplied,
                           -STEER_SLEW_PER_UPDATE, STEER_SLEW_PER_UPDATE);
  steerApplied += delta;
  // Avoid residual floating-point offsets at the actual center.
  if (target == 0.0f && fabs(steerApplied) < 0.0001f) steerApplied = 0.0f;

  if (fabs(delta) > DIRECTION_EPS)
    steerDirection = (delta > 0.0f) ? 1 : -1;

  float bias = 0.0f;
  if (steerDirection > 0) bias = BACKLASH_MOVING_POS;
  else if (steerDirection < 0) bias = -BACKLASH_MOVING_NEG;

  float axleSteering = clampFloat(steerApplied + bias, -1.0f, 1.0f);
  writeSteeringNormalized(axleSteering);
  writeSteeringWheelNormalized(steerApplied);
  applyBrake(braking);
}

float mphToERPM(float mph) {
  float feetPerMinute = mph * 5280.0f / 60.0f;
  float wheelCircumferenceFt = PI * (WHEEL_DIAMETER_IN / 12.0f);
  float wheelRPM = feetPerMinute / wheelCircumferenceFt;
  float motorRPM = wheelRPM * GEAR_RATIO;
  return motorRPM * MOTOR_POLE_PAIRS;
}

void updateVesc() {
  unsigned long now = millis();
  if (now - lastVescTxMs < VESC_TX_INTERVAL_MS) return;
  lastVescTxMs = now;
  VESC.setRPM(mphToERPM(speed));
}

void setup() {
  steer.attach(10);
  brake.attach(6);
  steeringWheel.attach(STEERING_WHEEL_PIN);
  Serial.begin(BAUD_RATE);

  steerApplied = 0.0f;
  writeSteeringNormalized(0.0f);
  writeSteeringWheelNormalized(0.0f);
  applyBrake(braking);

  vescSerial.begin(19200);
  VESC.setSerialPort(&vescSerial);
  lastValidMsgMs = millis();
}

void loop() {
  readAndParseSerial();
  checkStaleness();
  updateActuators();
  updateVesc();
}
