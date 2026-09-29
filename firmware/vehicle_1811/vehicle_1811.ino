// 1811 vehicle firmware -- Arduino Uno
//
// Receives {"speed","steering","braking"} JSON lines from the Karbon over the
// hardware UART and drives:
//   - steering servo (Docyke S350)  via ServoTimer2 on pin 10
//   - brake servo                   via ServoTimer2 on pin 6
//   - traction motor                via VESC UART on AltSoftSerial (pins 8/9)
//
// Steering is not written straight through. It passes through a deadband, a
// slew-rate limiter, and a directional backlash bias, all evaluated on a fixed
// 50 Hz tick. See "STEERING PIPELINE" below for why.

#include <AltSoftSerial.h>
#include <VescUart.h>
#include <ServoTimer2.h>

// AltSoftSerial has FIXED pins on Uno: TX=9, RX=8. Do not reassign.
AltSoftSerial vescSerial;
VescUart VESC;

ServoTimer2 steer;
ServoTimer2 brake;

#define BAUD_RATE        57600
#define MSG_BUF_SIZE     96
#define STALE_TIMEOUT_MS 250

const char* FORMAT_STRING = "{\"speed\": %f, \"steering\": %f, \"braking\": %f}";

// ---------------------------------------------------------------------------
// STEERING CALIBRATION -- measured on the vehicle
//
//   -25°       0°        +25°
//   733 µs    1150 µs    1567 µs
//
// so: µs = 1150.00 + 416.667 * steer, and 16.667 µs == 1 degree.
// Every extra ~16.667 µs in the Arduino pulse changes steering by ~1°.
//
// This is algebraically identical to the previous convertToMicro(117 + 75*angle)
// two-step formula, just written so the calibration is visible instead of
// buried in a 0..360 degree mapping the servo never used.
//
// DO NOT change these without re-measuring axle angle against pulse width. An
// earlier draft of this file used 1083.33/500 (from a stale 105 + 90*angle
// version) -- that commands 733-583 = 150 µs, or 9 degrees, of extra travel at
// full left lock, which can drive the linkage into its mechanical stop.
//
// The S350 accepts 0.5-2.5 ms per its datasheet, so 733 us is well in spec.
// ServoTimer2's stock MIN_PULSE_WIDTH is 750 -- just above full left lock,
// which is why the library copy in use here has been lowered to 500. With the
// stock library, full left would silently clamp.
// ---------------------------------------------------------------------------
const float STEER_CENTER_US = 1150.0f;
const float STEER_SPAN_US   = 416.667f;  // us per 1.0 of normalized steer
const float STEER_DEG_FULL  = 25.0f;     // degrees at steer = 1.0

const int SERVO_MIN_US = 500;           // Docyke S350 datasheet limits
const int SERVO_MAX_US = 2500;

// ---------------------------------------------------------------------------
// STEERING PIPELINE tuning
//
// Evaluated once per SERVO_UPDATE_MS, in this order:
//
//   target -> deadband -> slew limit -> [direction] -> backlash bias -> clamp
//
// SERVO_UPDATE_MS: the S350 latches a new position at 50 Hz, so writing faster
//   accomplishes nothing. Running the whole chain on a fixed tick also makes
//   the slew rate an actual rate (deg/s) instead of something that drifts
//   whenever the loop body changes.
//
// STEER_SLEW_PER_UPDATE: the current-draw fix. A step command asks a 34 N.m
//   actuator for maximum acceleration, and a *reversal* asks it to brake that
//   inertia and re-accelerate -- two near-stall current events back to back.
//   That is the voltage sag. Capping how far the command may move per tick
//   bounds the acceleration, and therefore the peak current.
//     0.08 per 20 ms tick = 2 deg / 20 ms = 100 deg/s (full lock to full lock
//     in ~0.5 s, roughly how fast a human turns a wheel). Lower it if sag
//     persists; raise it if steering feels sluggish.
//
// STEER_DEADBAND: pure_pursuit publishes at 30 Hz and jitters around zero near
//   center. Without this, the gearbox is driven back and forth through its own
//   backlash continuously.
//
// BACKLASH_MOVING_*: the S350's metal planetary gearset carries ~2 deg of lash,
//   and its magnetic encoder sits on the MOTOR side of that gearset -- so the
//   servo's own loop cannot see it. Measured: approaching 0 from -25 (moving
//   positive) lands at -2 deg; approaching from +25 (moving negative) lands
//   clean. So only positive-direction motion is compensated.
//   Expressed in normalized units: 2 deg / 25 deg = 0.08.
//
//   The bias is deliberately NOT slew-limited. While traversing the lash the
//   gear teeth are disengaged, so the motor is nearly unloaded and the jump
//   costs little current -- rate-limiting it would only delay take-up.
//
//   It is also deliberately held while stationary: the lash stays taken up on
//   whichever flank was last pushed against, so the offset must persist. This
//   is why it is a held bias and not an overshoot-then-return move; returning
//   would reverse direction and re-open the lash on the opposite flank,
//   landing 2 deg off the other way.
// ---------------------------------------------------------------------------
#define SERVO_UPDATE_MS 20              // 50 Hz -- matches the servo's latch rate

const float STEER_SLEW_PER_UPDATE = 0.08f;   // normalized units per tick
const float STEER_DEADBAND        = 0.02f;   // ignore requested changes below this
const float BACKLASH_MOVING_POS   = 0.08f;   // 2 deg, applied when moving toward +
const float BACKLASH_MOVING_NEG   = 0.00f;   // measured clean in this direction
const float DIRECTION_EPS         = 0.001f;  // below this, motion isn't real

// ---------------------------------------------------------------------------
// VESC
//
// A 10-byte setRPM packet at 19200 baud occupies 5.2 ms of wire time. Sending
// one per loop iteration saturated the link (~100% TX duty) and kept
// AltSoftSerial's Timer1 ISRs firing continuously, which jitters the Timer2
// servo pulses by ~0.2 deg. At 50 Hz the duty drops to ~26%; raise the
// interval to 50 ms (20 Hz, ~10% duty) if jitter is still visible.
//
// Do NOT drop below the VESC's own command timeout or it will cut the motor.
// ---------------------------------------------------------------------------
#define VESC_TX_INTERVAL_MS 20

const float WHEEL_DIAMETER_IN = 12;
const float GEAR_RATIO = 1.0;
const int MOTOR_POLE_PAIRS = 7;

char msgBuf[MSG_BUF_SIZE];

// Commanded values, straight off the wire.
float speed = 0.0;
float steering = 0;   // -1 to 1
float braking = 0;    // 0 to 1

// Steering pipeline state.
float steerApplied = 0.0f;   // slew-limited, UNBIASED. Direction is derived
                             // from this, never from the biased output --
                             // otherwise the bias feeds its own direction
                             // detection and self-oscillates.
int steerDirection = 0;      // +1 / -1 / 0. Starts at 0 = UNKNOWN: at power-on
                             // we have no idea which flank the lash is resting
                             // on, so no bias is applied until the first real
                             // motion establishes it. Defaulting to +/-1 here
                             // would offset the wheels by the backlash amount
                             // before anything had moved.
unsigned long lastServoUpdateMs = 0;

unsigned long lastVescTxMs = 0;
unsigned long lastValidMsgMs = 0;
bool linkStale = true;

void setup() {
  // brake moved from pin 9 -> pin 6, since AltSoftSerial owns pin 9 (TX) on Uno
  steer.attach(10);
  brake.attach(6);

  Serial.begin(BAUD_RATE);
  // readBytesUntil() blocks up to this long waiting for a newline. A 60-byte
  // command at 57600 takes 10.4 ms to arrive, so this must stay comfortably
  // above that or messages get truncated mid-line.
  Serial.setTimeout(20);

  steerApplied = 0.0f;
  writeSteeringNormalized(0.0f);
  applyBrake(braking);

  vescSerial.begin(19200);  // VESC UART, on AltSoftSerial's fixed pins 8/9
  VESC.setSerialPort(&vescSerial);

  lastValidMsgMs = millis();
}

void loop() {
  readAndParseSerial();
  checkStaleness();
  updateActuators();   // 50 Hz gated
  updateVesc();        // rate gated
}

// ---------------------------------------------------------------------------
// Serial command intake
// ---------------------------------------------------------------------------
void readAndParseSerial() {
  // Drain everything currently buffered, not just one line -- if multiple
  // messages queued up, we only care about the newest one anyway.
  bool gotValid = false;
  float parsedSpeed = 0, parsedSteering = 0, parsedBraking = 0;

  while (Serial.available() > 0) {
    int len = Serial.readBytesUntil('\n', msgBuf, MSG_BUF_SIZE - 1);
    if (len <= 0) continue;
    msgBuf[len] = '\0';

    // Parse into locals and commit only if all three fields came from THIS
    // line. parseField writes *out solely on success, so committing per-field
    // could otherwise pair speed from one message with steering from an older
    // one -- a combination nothing ever actually sent.
    float s, st, b;
    if (parseField(msgBuf, "\"speed\":", &s)
     && parseField(msgBuf, "\"steering\":", &st)
     && parseField(msgBuf, "\"braking\":", &b)) {
      parsedSpeed = s;
      parsedSteering = st;
      parsedBraking = b;
      gotValid = true;  // keep overwriting -- last valid line in the buffer wins
    }
  }

  if (gotValid) {
    speed = parsedSpeed;
    steering = parsedSteering;
    braking = parsedBraking;
    lastValidMsgMs = millis();
    linkStale = false;
  }
}

bool parseField(const char* buf, const char* key, float* out) {
  const char* found = strstr(buf, key);
  if (found == NULL) {
    return false;
  }
  *out = atof(found + strlen(key));
  return true;
}

void checkStaleness() {
  if (!linkStale && (millis() - lastValidMsgMs > STALE_TIMEOUT_MS)) {
    linkStale = true;
    speed = 0.0;
    steering = 0;
    // Deliberately left at 0 (coast, not brake) per current vehicle policy.
    // Note this means a dead link coasts rather than stops; the human on the
    // kill switch is the backstop.
    braking = 0;
    // steerApplied is intentionally NOT snapped to 0 here -- letting it slew
    // back through the normal path keeps the same current ceiling on the way
    // to center that every other move gets.
  }
}

// ---------------------------------------------------------------------------
// Actuator update -- fixed 50 Hz tick
// ---------------------------------------------------------------------------
void updateActuators() {
  unsigned long now = millis();
  if (now - lastServoUpdateMs < SERVO_UPDATE_MS) return;
  lastServoUpdateMs = now;

  float target = steering;

  // Deadband: hold position rather than chase sub-threshold noise.
  if (fabs(target - steerApplied) < STEER_DEADBAND) {
    target = steerApplied;
  }

  // Slew limit: bounds commanded acceleration, and therefore peak current.
  float delta = target - steerApplied;
  if (delta >  STEER_SLEW_PER_UPDATE) delta =  STEER_SLEW_PER_UPDATE;
  if (delta < -STEER_SLEW_PER_UPDATE) delta = -STEER_SLEW_PER_UPDATE;
  steerApplied += delta;

  // Direction of ACTUAL motion. Held when stationary so the backlash bias
  // persists while the servo holds position. Deriving it from the slewed
  // signal (rather than the raw 30 Hz target) is also what keeps it from
  // chattering near center.
  if (fabs(delta) > DIRECTION_EPS) {
    steerDirection = (delta > 0.0f) ? 1 : -1;
  }

  // Backlash bias -- applied as a step, see notes at the top. Direction 0
  // (nothing has moved since boot) means no bias.
  float bias = 0.0f;
  if (steerDirection > 0)      bias =  BACKLASH_MOVING_POS;
  else if (steerDirection < 0) bias = -BACKLASH_MOVING_NEG;
  float biased = steerApplied + bias;

  // Clamp LAST, so the bias can never push the command past full lock.
  // (Consequence: at full lock the bias is clamped away and backlash is
  // uncompensated there. Accepted -- it matters least at the extremes.)
  if (biased >  1.0f) biased =  1.0f;
  if (biased < -1.0f) biased = -1.0f;

  writeSteeringNormalized(biased);

  // Brake is NOT slew limited: braking should take effect immediately.
  applyBrake(braking);
}

// ---------------------------------------------------------------------------
// Output conversion
// ---------------------------------------------------------------------------
void writeSteeringNormalized(float s) {
  float us = STEER_CENTER_US + STEER_SPAN_US * s;
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;   // datasheet backstop
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  steer.write((int)us);   // >180 -> ServoTimer2 treats this as microseconds
}

int convertToMicro(float angle) {
  return (int)(500.0 + (angle / 360.0) * 2000.0);
}

void applyBrake(float angle) {
  // Left on the original mapping -- this is calibrated to the brake linkage
  // and nothing in this change touches braking geometry.
  float actAngle = 300 - (70 * angle);
  int us = convertToMicro(actAngle);
  brake.write(us);
}

// ---------------------------------------------------------------------------
// VESC
// ---------------------------------------------------------------------------
void updateVesc() {
  unsigned long now = millis();
  if (now - lastVescTxMs < VESC_TX_INTERVAL_MS) return;
  lastVescTxMs = now;

  VESC.setRPM(mphToERPM(speed));
}

float mphToERPM(float mph) {
  float feetPerMinute = mph * 5280.0 / 60.0;
  float wheelCircumferenceFt = PI * (WHEEL_DIAMETER_IN / 12.0);
  float wheelRPM = feetPerMinute / wheelCircumferenceFt;
  float motorRPM = wheelRPM * GEAR_RATIO;
  return motorRPM * MOTOR_POLE_PAIRS;
}

// ---------------------------------------------------------------------------
// VESC telemetry -- DISABLED
//
// Two reasons this is off:
//
// 1. Serial.print() here writes to the SAME UART that receives commands. The
//    host parses that stream as JSON, so debug lines corrupt it (this was
//    found and fixed once already; it came back). If telemetry is re-enabled,
//    it must NOT go to Serial.
//
// 2. getVescValues() is a blocking request/response. The ~78-byte reply at
//    19200 baud is 40.6 ms of wire time, which pinned the whole loop to ~17 Hz
//    while the host publishes at 30 Hz -- half the commands were being dropped,
//    and the larger per-update steering steps that resulted made the current
//    transients worse.
//
// To bring it back safely: put it on its own ~10 Hz timer (never per-loop),
// and send the values to the host as a proper JSON line the ROS side can
// parse, rather than as free-text prints.
// ---------------------------------------------------------------------------
// void readVescData() {
//   if (VESC.getVescValues()) {
//     float rpm = VESC.data.rpm;
//     float current = VESC.data.avgMotorCurrent;
//     float dutyCycle = VESC.data.dutyCycleNow;
//   }
// }
