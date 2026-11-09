// Arduino Uno vehicle firmware: Docyke S350 + HS-422 steering wheel
// Input: newline-terminated JSON {"speed":..., "steering":..., "braking":...}
// Hardware Serial: 57600 baud from Karbon controller
// ServoTimer2: Docyke S350 on D10, brake on D6, HS-422 on D3
// AltSoftSerial: VESC RX D8 / TX D9, 19200 baud
//
// TEST SAFELY: lift wheels, disconnect traction power / steering-wheel linkage
// and use a regulated external 4.8-6 V supply for HS-422. Connect ALL grounds.

#include <AltSoftSerial.h>
#include <VescUart.h>
#include <ServoTimer2.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

AltSoftSerial vescSerial;
VescUart VESC;
ServoTimer2 steer;
ServoTimer2 brake;
ServoTimer2 steeringWheel;

const int DOCYKE_PIN = 10;
const int BRAKE_PIN = 6;
const int STEERING_WHEEL_PIN = 3;

const unsigned long BAUD_RATE = 57600;
const size_t MSG_BUF_SIZE = 96;
const unsigned long STALE_TIMEOUT_MS = 250;
const unsigned long SERVO_UPDATE_MS = 20; // 50 Hz
const unsigned long VESC_TX_INTERVAL_MS = 20;

// Docyke S350 calibration (measured on the vehicle):
// -25 deg -> 733 us; center -> 1150 us; +25 deg -> 1567 us.
const float STEER_CENTER_US = 1150.0f;
const float STEER_SPAN_US = 416.667f;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// HS-422: reversing the sign makes it turn opposite to the previous firmware.
// If it now turns opposite to the Docyke physically, change true to false.
const bool REVERSE_HITEC = true;
// Start with a smaller travel range (about +/- 54 nominal degrees) to avoid stops.
// Increase toward 1000/2000 only after confirming safe free travel.
const int HITEC_MIN_US = 1200;
const int HITEC_CENTER_US = 1500;
const int HITEC_MAX_US = 1800;

const float STEER_SLEW_PER_UPDATE = 0.08f;
const float STEER_DEADBAND = 0.02f;
const float BACKLASH_MOVING_POS = 0.08f;
const float BACKLASH_MOVING_NEG = 0.00f;
const float DIRECTION_EPS = 0.001f;

const float WHEEL_DIAMETER_IN = 12.0f;
const float GEAR_RATIO = 1.0f;
const int MOTOR_POLE_PAIRS = 7;

char msgBuf[MSG_BUF_SIZE];
float speed = 0.0f;
float steering = 0.0f;
float braking = 0.0f;
float steerApplied = 0.0f;
int steerDirection = 0;
unsigned long lastServoUpdateMs = 0;
unsigned long lastVescTxMs = 0;
unsigned long lastValidMsgMs = 0;
bool linkStale = true;

void writeSteeringNormalized(float s);
void writeSteeringWheelNormalized(float s);
void applyBrake(float amount);
void readAndParseSerial();
bool parseField(const char *buf, const char *key, float *out);
void checkStaleness();
void updateActuators();
void updateVesc();
float mphToERPM(float mph);

void setup() {
  steer.attach(DOCYKE_PIN);
  brake.attach(BRAKE_PIN);
  steeringWheel.attach(STEERING_WHEEL_PIN);

  Serial.begin(BAUD_RATE);
  Serial.setTimeout(20);

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

void readAndParseSerial() {
  bool gotValid = false;
  float parsedSpeed = 0.0f, parsedSteering = 0.0f, parsedBraking = 0.0f;

  while (Serial.available() > 0) {
    int len = Serial.readBytesUntil('\n', msgBuf, MSG_BUF_SIZE - 1);
    if (len <= 0) continue;
    msgBuf[len] = '\0';

    float s, st, b;
    if (parseField(msgBuf, "\"speed\":", &s) &&
        parseField(msgBuf, "\"steering\":", &st) &&
        parseField(msgBuf, "\"braking\":", &b) &&
        isfinite(s) && isfinite(st) && isfinite(b)) {
      parsedSpeed = s;
      parsedSteering = st;
      parsedBraking = b;
      gotValid = true;
    }
  }

  if (gotValid) {
    speed = parsedSpeed;
    steering = constrain(parsedSteering, -1.0f, 1.0f);
    braking = constrain(parsedBraking, 0.0f, 1.0f);
    lastValidMsgMs = millis();
    linkStale = false;
  }
}

bool parseField(const char *buf, const char *key, float *out) {
  const char *found = strstr(buf, key);
  if (found == NULL) return false;
  *out = atof(found + strlen(key));
  return true;
}

void checkStaleness() {
  if (!linkStale && millis() - lastValidMsgMs > STALE_TIMEOUT_MS) {
    linkStale = true;
    speed = 0.0f;
    steering = 0.0f;
    braking = 0.0f; // Preserve original vehicle coast-on-stale policy.
  }
}

void updateActuators() {
  unsigned long now = millis();
  if (now - lastServoUpdateMs < SERVO_UPDATE_MS) return;
  lastServoUpdateMs = now;

  float target = steering;
  if (fabs(target - steerApplied) < STEER_DEADBAND) target = steerApplied;

  float delta = target - steerApplied;
  delta = constrain(delta, -STEER_SLEW_PER_UPDATE, STEER_SLEW_PER_UPDATE);
  steerApplied += delta;

  if (fabs(delta) > DIRECTION_EPS)
    steerDirection = (delta > 0.0f) ? 1 : -1;

  float bias = 0.0f;
  if (steerDirection > 0) bias = BACKLASH_MOVING_POS;
  else if (steerDirection < 0) bias = -BACKLASH_MOVING_NEG;

  float biased = constrain(steerApplied + bias, -1.0f, 1.0f);

  // Both servos follow the SAME compensated steering command.
  writeSteeringNormalized(biased);
  writeSteeringWheelNormalized(biased);
  applyBrake(braking);
}

void writeSteeringNormalized(float s) {
  float us = STEER_CENTER_US + STEER_SPAN_US * s;
  us = constrain(us, (float)SERVO_MIN_US, (float)SERVO_MAX_US);
  steer.write((int)us);
}

void writeSteeringWheelNormalized(float s) {
  s = constrain(s, -1.0f, 1.0f);
  float sign = REVERSE_HITEC ? -1.0f : 1.0f;
  // Support asymmetric safe endpoints if subsequently calibrated.
  float span = (sign * s >= 0.0f)
      ? (HITEC_MAX_US - HITEC_CENTER_US)
      : (HITEC_CENTER_US - HITEC_MIN_US);
  float us = HITEC_CENTER_US + sign * s * span;
  us = constrain(us, (float)HITEC_MIN_US, (float)HITEC_MAX_US);
  steeringWheel.write((int)us);
}

int convertToMicro(float angle) {
  return (int)(500.0f + (angle / 360.0f) * 2000.0f);
}

void applyBrake(float amount) {
  float actAngle = 300.0f - 70.0f * amount;
  brake.write(convertToMicro(actAngle));
}

void updateVesc() {
  unsigned long now = millis();
  if (now - lastVescTxMs < VESC_TX_INTERVAL_MS) return;
  lastVescTxMs = now;
  VESC.setRPM(mphToERPM(speed));
}

float mphToERPM(float mph) {
  float feetPerMinute = mph * 5280.0f / 60.0f;
  float wheelCircumferenceFt = PI * (WHEEL_DIAMETER_IN / 12.0f);
  float wheelRPM = feetPerMinute / wheelCircumferenceFt;
  float motorRPM = wheelRPM * GEAR_RATIO;
  return motorRPM * MOTOR_POLE_PAIRS;
}
