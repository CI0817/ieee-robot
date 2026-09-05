#include <Arduino.h>
#include "BluetoothSerial.h"

// Classic Bluetooth SPP -- shows up as a virtual serial port once paired,
// so telemetry works untethered. Built into the ESP32, no extra hardware.
// If this fails to compile with a "Bluetooth is not enabled" error, the
// Arduino-ESP32 core build in use has BT disabled at the sdkconfig level --
// not expected on a stock esp32dev board/core, but worth knowing why.
BluetoothSerial SerialBT;

// Per-sensor calibration values measured on this robot.
constexpr int LEFT_BLACK_THRESHOLD = 330;
constexpr int MIDDLE_BLACK_THRESHOLD = 60;
constexpr int RIGHT_BLACK_THRESHOLD = 275;

constexpr int LEFT_BLACK_MAX = 620;
constexpr int MIDDLE_BLACK_MAX = 105;
constexpr int RIGHT_BLACK_MAX = 520;
constexpr int MAX_PWM = 160;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;

// Sharp-turn pivot override: if one outer sensor is very strongly on black
// while the other is essentially clear, override the smooth P-controller
// with a one-sided pivot instead of relying on max_correction alone (which
// can never drive a wheel below a positive floor). Enter/exit thresholds
// are deliberately different (hysteresis), and entry requires several
// consecutive ticks (debounce), so a single noisy sample near the boundary
// can't flip the robot in and out of pivot mode.
// NOTE: starting-point values -- re-tune with the calibration script
// against the current (rebuilt) sensor mount before trusting these.
constexpr float PIVOT_ENTER_BLACK = 0.55f;
constexpr float PIVOT_ENTER_CLEAR = 0.10f;
constexpr float PIVOT_EXIT_BLACK = 0.35f;
// Raised from 3 -> 15 (~60ms -> ~300ms of required sustained saturation).
// A gentle curve can briefly saturate an outer sensor to 1.0 too (confirmed
// in testing), so blackness *level* alone can't tell a curve from a sharp
// corner -- only a real corner sustains it this long. Retune once tested
// against an actual sharp corner: if real corners now feel slow to trigger,
// lower this; if curves still trip pivot, raise it further.
constexpr int PIVOT_DEBOUNCE_TICKS = 15;

// Measured geometry: sensor array is ~6cm ahead of the wheel axle, which is
// the point a point-turn actually rotates about (equal-and-opposite wheel
// speeds cancel translation there). Pivoting the instant a corner is
// confirmed above therefore rotates around a point ~6cm short of the
// corner's real vertex. Worked example (square corner, robot driving +x,
// new line continuing +y): pivoting immediately lands the sensor array at
// roughly (-6, +6) while the new line is at x=0 -- a 6cm sideways miss,
// nearly double the full 3.5cm sensor spread, so not just imprecision.
// Creeping straight first for roughly SENSOR_TO_AXLE_CM brings the axle up
// to the vertex, so the same point-turn then rotates around approximately
// the right point and the sensor lands back on the new line instead of
// beside it. See chat for the full derivation.
constexpr float SENSOR_TO_AXLE_CM = 6.0f;

// GUESS -- not measured yet. Converts SENSOR_TO_AXLE_CM into a creep
// duration at straight_speed via an assumed ~30cm/s. Replace
// ASSUMED_CREEP_SPEED_CM_PER_S with a real measurement (time the robot over
// a fixed distance at this PWM -- see the calibration test) and recompute,
// then fine-tune further by watching where the array actually lands after
// a real 90-degree corner: too short undershoots the vertex (still a
// sideways miss, just smaller); too long overshoots past it (misses the
// new line on the other side, or drives onto blank floor first).
constexpr float ASSUMED_CREEP_SPEED_CM_PER_S = 30.0f;
constexpr unsigned long PIVOT_CREEP_MS = static_cast<unsigned long>(
    (SENSOR_TO_AXLE_CM / ASSUMED_CREEP_SPEED_CM_PER_S) * 1000.0f);

// PID steering gains, applied to line_error, a weighted centroid across all
// three sensors (range -1..1; positive means the line is toward the right
// sensor). Using all three -- not just outer-sensor difference -- matters
// because the outer sensors are near-binary (see MAX_SPEED_STEP_PER_TICK
// comment below): on a gentle curve the line sits under the middle sensor
// most of the time, and a left/right-only error is pinned at 0 there,
// producing a straight-then-snap-correct hunt instead of a smooth arc.
// KD is new -- previously P-only, deliberately deferred until the pivot
// override above was validated against a real corner. Starting guess only,
// re-tune alongside the pivot constants once on hardware: too much KD will
// amplify sensor noise into visible wheel jitter on straight sections.
constexpr float LINE_KP = 80.0f;
constexpr float LINE_KD = 0.0f; // zeroed for KP-only / pivot tuning pass -- see chat

// Fixed control-loop cadence. Without this, loop() runs as fast as the MCU
// can execute it (sub-millisecond), which makes dt in the D-term tiny and
// noisy -- (line_error - prev_line_error) divided by a near-zero dt turns
// ordinary ADC noise into huge derivative spikes every tick (the "very very
// jittery" symptom). It also makes PIVOT_DEBOUNCE_TICKS represent a real
// ~60ms window instead of however fast the raw loop happens to spin.
constexpr unsigned long CONTROL_INTERVAL_MS = 20;

// Flip off once KP/KD are settled -- see the throttled print at the end of
// pid_drive() for why this shouldn't just be left on permanently.
constexpr bool ENABLE_TUNING_TELEMETRY = true;

// Slew-rate limit: max change in a wheel's commanded speed per control
// tick, applied in drive_motors() so it smooths every path uniformly (P/D
// corrections, pivot entry/exit, gap-hold). Outer IR sensors appear to have
// a near-binary black/white response (jump from ~0 to full saturation over
// a tiny physical offset -- confirmed by hunting that persisted even after
// lowering straight_speed), so the *input* signal itself is step-like no
// matter how KP/KD/speed are tuned. This smooths the *output* instead:
// turns a sudden correction jump into a fast ramp. Starting guess -- retune
// alongside everything else; too low will feel sluggish into real corners.
// Lowered from 40 -> 8: at 40/tick (20ms tick), a full ~80-unit correction
// swing completed in 2 ticks = 40ms -- basically instantaneous to the eye,
// so it wasn't actually producing a perceptible ramp. At 8/tick the same
// swing takes ~10 ticks = ~200ms, which should actually read as smooth.
constexpr int MAX_SPEED_STEP_PER_TICK = 8;

// Safety cutoff: if no sensor has seen black for this long, stop instead of
// coasting on a held command forever (e.g. a stale pivot command with no
// line in sight would otherwise spin in place indefinitely).
// TEMPORARY: bumped to 5s to unblock testing dash gaps (up to 5cm) without
// measuring real cruise speed yet. At 5s, a genuine line loss (missed a
// corner, drove off-track) coasts blind for a long real distance before
// stopping -- replace with a properly measured value (gap_length / speed,
// with margin) before any real practice/competition run.
constexpr unsigned long MAX_BLIND_MS = 5000;

const int turning_speed = 100;
const int straight_speed = 90; // lowered from 100 to test the hunting-on-curves hypothesis -- see chat

const int left_ir = 32;
const int middle_ir = 35;
const int right_ir = 34;
const int left_motorA = 19;
const int left_motorB = 18;
const int right_motorA = 17;
const int right_motorB = 5;
const int left_pwm = 21;
const int right_pwm = 16;

// If a motor spins backward when the rest of the code commands it forward
// (e.g. after a rewire swapped its leads), flip its constant to -1 here
// rather than touching any of the steering/turning logic above -- this is
// the only place physical motor polarity is corrected.
constexpr int LEFT_MOTOR_DIRECTION = 1;
constexpr int RIGHT_MOTOR_DIRECTION = 1;

bool capacitor_zone = false;

// Last commanded wheel speeds, so a brief line loss (a dash gap) can hold
// the previous steering command instead of resetting to a default.
int last_left_speed = 0;
int last_right_speed = 0;

// Pivot-mode state, persisted across loop() calls for the debounce/hysteresis above.
int8_t pivot_state = 0; // 0 = not pivoting, -1 = pivoting left, 1 = pivoting right
int8_t pivot_candidate_side = 0;
int pivot_candidate_ticks = 0;

// Corner-creep state (see SENSOR_TO_AXLE_CM/PIVOT_CREEP_MS above): 0 = not
// creeping, otherwise the direction (-1/1) the confirmed corner will pivot
// once the creep finishes. Runs open-loop on a timer rather than sensor
// feedback, since the line is expected to disappear under every sensor for
// some or all of this phase.
int8_t corner_creep_direction = 0;
unsigned long corner_creep_started_ms = 0;

// Tracks how long the line has been out of view, for the MAX_BLIND_MS cutoff.
bool line_was_visible = true;
unsigned long line_lost_since_ms = 0;

// D-term state, persisted across pid_drive() calls. dt is measured from
// wall-clock time (not assumed constant per loop) so the derivative stays
// correct across a pivot-mode detour or a brief line-loss coast, both of
// which skip line_error updates for a while without calling pid_drive.
float prev_line_error = 0.0f;
unsigned long prev_error_ms = 0;
bool have_prev_error = false;

int determine_drive_mode();
void pid_drive();
void drive_pivot(int8_t direction);
void drive_motors(int left_vel, int right_vel);
bool check_black(int sensor_pin);
int black_threshold_for(int sensor_pin);
void end_zone();
void stop();

void setup()
{
  pinMode(left_ir, INPUT);
  pinMode(middle_ir, INPUT);
  pinMode(right_ir, INPUT);
  pinMode(left_motorA, OUTPUT);
  pinMode(right_motorA, OUTPUT);
  pinMode(left_motorB, OUTPUT);
  pinMode(right_motorB, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(right_pwm, OUTPUT);

  // Explicitly attach the motor-enable pins to ESP32 LEDC PWM outputs.
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(left_pwm, LEFT_PWM_CHANNEL);
  ledcAttachPin(right_pwm, RIGHT_PWM_CHANNEL);
  Serial.begin(9600);
  SerialBT.begin("IEEE-Robot"); // device name shown when pairing/scanning
}

void loop()
{
  // drive_motors(50, 50);

  // Gate the control loop to a fixed cadence -- see CONTROL_INTERVAL_MS
  // above for why this matters for the D-term and the pivot debounce.
  static unsigned long last_tick_ms = 0;
  const unsigned long now_ms = millis();
  if (now_ms - last_tick_ms < CONTROL_INTERVAL_MS)
  {
    return;
  }
  last_tick_ms = now_ms;

  determine_drive_mode();
}

int determine_drive_mode()
{
  const bool left_black = check_black(left_ir);
  const bool middle_black = check_black(middle_ir);
  const bool right_black = check_black(right_ir);

  if (left_black || middle_black || right_black ||
      pivot_state != 0 || corner_creep_direction != 0)
  {
    // Any sensor seeing black is enough to steer from -- pid_drive() reads
    // the raw analog values itself and decides between smooth steering and
    // the pivot override. A confirmed corner maneuver (creep then pivot) is
    // also routed here even with every sensor reading white, since it runs
    // open-loop on its own timer and is expected to lose the line for some
    // or all of it -- see SENSOR_TO_AXLE_CM above.
    line_was_visible = true;
    pid_drive();
    return 1;
  }

  if (capacitor_zone)
  {
    // TODO: capacitor_zone is never set true anywhere yet -- this whole
    // branch is currently unreachable until that trigger is designed.
    drive_motors(straight_speed, straight_speed);
    while (!check_black(left_ir) &&
           !check_black(middle_ir) &&
           !check_black(right_ir))
    {
      delay(10);
    }
    stop();
    return 5; // Straight drive mode in capacitor zone
  }

  // Line lost: hold the last command briefly (covers a short dash gap)
  // rather than snapping to a default speed, but don't coast forever --
  // give up and stop if it's been lost too long.
  if (line_was_visible)
  {
    line_was_visible = false;
    line_lost_since_ms = millis();
  }

  if (millis() - line_lost_since_ms > MAX_BLIND_MS)
  {
    stop();
    return 6; // Gave up waiting to reacquire the line
  }

  drive_motors(last_left_speed, last_right_speed);
  return 0; // Coasting on last known command, line not yet reacquired
}

void pid_drive()
{
  // Serial.print("\nPID Mode");
  capacitor_zone = false;

  // Shared by both the pivot and P/D telemetry prints below, so pivot mode
  // (which used to return before ever reaching a print) is now visible too.
  const unsigned long now_ms = millis();
  static unsigned long last_print_ms = 0;
  const bool should_print =
      ENABLE_TUNING_TELEMETRY && (now_ms - last_print_ms >= 100);

  const int left_value = analogRead(left_ir);
  const int middle_value = analogRead(middle_ir);
  const int right_value = analogRead(right_ir);

  const int left_sensor_range = LEFT_BLACK_MAX - LEFT_BLACK_THRESHOLD;
  const int middle_sensor_range = MIDDLE_BLACK_MAX - MIDDLE_BLACK_THRESHOLD;
  const int right_sensor_range = RIGHT_BLACK_MAX - RIGHT_BLACK_THRESHOLD;

  const int left_error = constrain(
      left_value - LEFT_BLACK_THRESHOLD, 0, left_sensor_range);

  const int middle_error = constrain(
      middle_value - MIDDLE_BLACK_THRESHOLD, 0, middle_sensor_range);

  const int right_error = constrain(
      right_value - RIGHT_BLACK_THRESHOLD, 0, right_sensor_range);

  const float left_black =
      left_error / static_cast<float>(left_sensor_range);
  const float middle_black =
      middle_error / static_cast<float>(middle_sensor_range);
  const float right_black =
      right_error / static_cast<float>(right_sensor_range);

  // Corner creep: a confirmed corner (below) doesn't pivot immediately --
  // it first drives straight, open-loop, for PIVOT_CREEP_MS to bring the
  // axle up to the corner's vertex before the point-turn starts rotating
  // about it. See SENSOR_TO_AXLE_CM above for why pivoting immediately
  // would rotate about the wrong point.
  if (corner_creep_direction != 0)
  {
    if (now_ms - corner_creep_started_ms < PIVOT_CREEP_MS)
    {
      // Deliberately ignoring line_error here, not just because there may
      // not be one -- the point of this phase is to cover a fixed distance
      // dead-reckoned, so reacting to a fading/ambiguous reading would
      // undermine that.
      drive_motors(straight_speed, straight_speed);

      if (should_print)
      {
        last_print_ms = now_ms;
        char buf[80];
        snprintf(buf, sizeof(buf), "CREEP dir=%d t=%lu L=%d R=%d",
                 corner_creep_direction, now_ms - corner_creep_started_ms,
                 last_left_speed, last_right_speed);
        Serial.println(buf);
        SerialBT.println(buf);
      }
      return;
    }

    // Creep distance covered -- the axle should now be roughly at the
    // corner vertex, so hand off straight into the point-turn. Skips the
    // entry/exit-check block below on this transition tick (rather than
    // falling into the pivot_state != 0 branch further down) because
    // line_error right after a creep is likely near zero -- sensor
    // probably isn't over the line yet, per the geometry above -- and that
    // would otherwise read as an immediate (bogus) exit condition before a
    // single rotate command was ever issued.
    pivot_state = corner_creep_direction;
    corner_creep_direction = 0;
    have_prev_error = false; // don't derive across the creep+corner gap once KD is back on
    drive_pivot(pivot_state);

    if (should_print)
    {
      last_print_ms = now_ms;
      char buf[80];
      snprintf(buf, sizeof(buf), "PIVOT-START side=%d", pivot_state);
      Serial.println(buf);
      SerialBT.println(buf);
    }
    return;
  }

  // Sharp-turn pivot override -- see the PIVOT_* constants up top for why
  // this is debounced and hysteretic rather than a plain threshold check.
  if (pivot_state == 0)
  {
    int8_t candidate = 0;
    if (left_black >= PIVOT_ENTER_BLACK && right_black <= PIVOT_ENTER_CLEAR)
    {
      candidate = -1;
    }
    else if (right_black >= PIVOT_ENTER_BLACK && left_black <= PIVOT_ENTER_CLEAR)
    {
      candidate = 1;
    }

    if (candidate != 0 && candidate == pivot_candidate_side)
    {
      pivot_candidate_ticks++;
    }
    else
    {
      pivot_candidate_side = candidate;
      pivot_candidate_ticks = (candidate != 0) ? 1 : 0;
    }

    if (pivot_candidate_ticks >= PIVOT_DEBOUNCE_TICKS)
    {
      // Confirmed corner -- start the creep phase above rather than
      // pivoting this tick.
      corner_creep_direction = pivot_candidate_side;
      corner_creep_started_ms = now_ms;
      pivot_candidate_side = 0;
      pivot_candidate_ticks = 0;
    }
  }
  else
  {
    // Already pivoting: use the lower exit threshold so a momentary dip
    // right at the boundary doesn't bounce us in and out of pivot mode.
    const float active_black = (pivot_state < 0) ? left_black : right_black;
    if (active_black < PIVOT_EXIT_BLACK)
    {
      pivot_state = 0;
      pivot_candidate_side = 0;
      pivot_candidate_ticks = 0;
    }
  }

  if (pivot_state != 0)
  {
    drive_pivot(pivot_state);

    if (should_print)
    {
      last_print_ms = now_ms;
      char buf[80];
      // L/R here are last_left_speed/last_right_speed, i.e. the actual
      // post-slew-limit speed drive_motors() just applied, not the raw
      // pivot target -- so this matches what the motors really did.
      snprintf(buf, sizeof(buf), "PIVOT side=%d L_blk=%.3f R_blk=%.3f L=%d R=%d",
               pivot_state, left_black, right_black, last_left_speed, last_right_speed);
      Serial.println(buf);
      SerialBT.println(buf);
    }
    return;
  }

  // Weighted centroid across all three sensors, at positions -1 (left), 0
  // (middle), +1 (right), normalized by total activation. The middle term
  // drops out of the numerator (weight 0) but still counts in the
  // denominator, so a line sitting mostly under the middle sensor pulls the
  // magnitude of line_error toward 0 (correctly reads as centered) instead
  // of being ignored outright -- see the LINE_KP comment above for why
  // outer-sensor-only difference couldn't do this.
  // Left black -> negative error -> left slows, right speeds up.
  // Right black -> positive error -> left speeds up, right slows.
  const float total_black = left_black + middle_black + right_black;
  const float line_error = (total_black > 0.05f)
                                ? (right_black - left_black) / total_black
                                : (right_black - left_black);

  const float dt = have_prev_error
                        ? (now_ms - prev_error_ms) / 1000.0f
                        : 0.0f;
  const float derivative = (dt > 0.0f)
                                ? (line_error - prev_line_error) / dt
                                : 0.0f;

  const int correction = static_cast<int>(
      line_error * LINE_KP + derivative * LINE_KD);

  prev_line_error = line_error;
  prev_error_ms = now_ms;
  have_prev_error = true;

  const int left_speed = constrain(
      straight_speed + correction, -MAX_PWM, MAX_PWM);

  const int right_speed = constrain(
      straight_speed - correction, -MAX_PWM, MAX_PWM);

  drive_motors(left_speed, right_speed);

  // Throttled: printing every loop tick at 9600 baud would block long
  // enough to distort the dt the D-term relies on. Flip
  // ENABLE_TUNING_TELEMETRY off once KP/KD are settled -- this is a
  // tuning aid, not permanent logging.
  if (should_print)
  {
    last_print_ms = now_ms;
    char buf[80];
    // L/R are last_left_speed/last_right_speed (the actual post-slew-limit
    // speed drive_motors() just applied), not the raw left_speed/right_speed
    // computed above -- so this matches what the motors really did.
    snprintf(buf, sizeof(buf), "err=%.3f d=%.3f corr=%d L=%d R=%d",
             line_error, derivative, correction, last_left_speed, last_right_speed);
    Serial.println(buf);
    SerialBT.println(buf);
  }
}

void drive_pivot(int8_t direction)
{
  // True point-turn: both wheels drive, opposite directions, instead of
  // reversing one wheel while parking the other. The parked-wheel version
  // only had one motor's worth of angular authority, which on a sharp
  // corner could be too slow to sweep the sensor array back onto the line
  // before MAX_BLIND_MS gives up and stops -- looking like the robot
  // freezing mid-turn. This roughly doubles turn rate for the same
  // turning_speed; re-check turning_speed once this lands on hardware,
  // since a faster pivot may now want a lower speed to avoid overshoot.
  if (direction < 0)
  {
    drive_motors(-turning_speed, turning_speed);
  }
  else
  {
    drive_motors(turning_speed, -turning_speed);
  }
}

void set_motor(int pwm_pin, int direction_pin_1, int direction_pin_2, int velocity)
{
  // Everything upstream (steering, pivot, hold-last-command) reasons about
  // velocity in logical terms: positive = forward. Physical wiring polarity
  // is corrected right here, in one place, before it becomes a direction pin.
  const bool is_left_motor = (pwm_pin == left_pwm);
  const int direction_multiplier =
      is_left_motor ? LEFT_MOTOR_DIRECTION : RIGHT_MOTOR_DIRECTION;

  velocity = constrain(velocity * direction_multiplier, -MAX_PWM, MAX_PWM);

  const int pwm = abs(velocity);

  if (velocity > 0)
  {
    digitalWrite(direction_pin_1, HIGH);
    digitalWrite(direction_pin_2, LOW);
  }
  else if (velocity < 0)
  {
    digitalWrite(direction_pin_1, LOW);
    digitalWrite(direction_pin_2, HIGH);
  }
  else
  {
    // Coast; use HIGH/HIGH instead if your driver supports braking.
    digitalWrite(direction_pin_1, LOW);
    digitalWrite(direction_pin_2, LOW);
  }

  const uint8_t pwm_channel =
      is_left_motor ? LEFT_PWM_CHANNEL : RIGHT_PWM_CHANNEL;
  ledcWrite(pwm_channel, pwm);
}

void drive_motors(int left_vel, int right_vel)
{
  // Slew-rate limit against the last actually-applied speed -- see
  // MAX_SPEED_STEP_PER_TICK above. A repeated call with the same value
  // (e.g. determine_drive_mode()'s gap-hold re-issuing last_left_speed) is
  // a no-op here since the requested change is already zero.
  left_vel = constrain(left_vel,
                        last_left_speed - MAX_SPEED_STEP_PER_TICK,
                        last_left_speed + MAX_SPEED_STEP_PER_TICK);
  right_vel = constrain(right_vel,
                         last_right_speed - MAX_SPEED_STEP_PER_TICK,
                         last_right_speed + MAX_SPEED_STEP_PER_TICK);

  last_left_speed = left_vel;
  last_right_speed = right_vel;

  set_motor(left_pwm, left_motorA, left_motorB, left_vel);
  set_motor(right_pwm, right_motorA, right_motorB, right_vel);
}

bool check_black(int sensor_pin)
{
  int sensor_value = analogRead(sensor_pin);
  return sensor_value > black_threshold_for(sensor_pin);
}

int black_threshold_for(int sensor_pin)
{
  if (sensor_pin == left_ir)
  {
    return LEFT_BLACK_THRESHOLD;
  }

  if (sensor_pin == middle_ir)
  {
    return MIDDLE_BLACK_THRESHOLD;
  }

  return RIGHT_BLACK_THRESHOLD;
}

void end_zone()
{
  capacitor_zone = false; // Reset capacitor zone flag
  stop();                 // Stop the motors
  // Additional logic for end zone can be added here
}

void stop()
{
  // Bypass the slew-rate limiter -- an explicit stop must be immediate,
  // not ramped, for safety (e.g. the MAX_BLIND_MS give-up case, or later
  // when stopping in the end box). Zeroing the reference point here means
  // drive_motors()'s constrain() has nothing to ramp from.
  last_left_speed = 0;
  last_right_speed = 0;
  drive_motors(0, 0); // Stop the motors
}
