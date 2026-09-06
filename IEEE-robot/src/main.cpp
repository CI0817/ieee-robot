#include <Arduino.h>
#include <BallRetrieve.h>

// Pivot only when one side sees a very strong line and the other is mostly clear.
constexpr float PIVOT_BLACK_LEVEL = 0.80f;
constexpr float PIVOT_OTHER_SIDE_MAX = 0.30f;
constexpr float TRIPLE_TURN_SCALE = 3.0f;

// After a long stretch with no sensor near the line (e.g. drifting across a
// wide gap or rounded feature), a single outer sensor suddenly reading
// strong black while the other outer is still clear looks identical to a
// genuine sharp corner - but on a wide curve it's just the robot drifting
// into the inside edge of the tape. Ignore that lone reading and keep
// driving straight until a second sensor (middle or the other outer)
// confirms the line is genuinely there before resuming normal steering.
constexpr uint32_t LINE_REACQUIRE_TIMEOUT_MS = 2000;
uint32_t last_genuine_line_ms = 0;
bool ignoring_lone_outer_hit = false;

int last_left_speed = 0;
int last_right_speed = 0;
int previous_black_sensor_count = 0;
bool triple_black_active = false;
bool triple_black_is_corner = false;

constexpr float TRIPLE_BLACK_SPEED_SCALE = 0.35f;
constexpr int MIN_TRIPLE_MOVING_PWM = 40;
constexpr float WHITE_GAP_SPEED_SCALE = 0.70f; //######################################################################################################################
// Fraction of each calibrated black range required to count as “detected.”
// Deliberately lenient - this only needs to catch a thin printed line,
// including its fainter edges, for ordinary line following.
constexpr float DETECT_BLACK_LEVEL = 0.15f;
// Fraction required to count as "genuinely black" for end-zone detection
// specifically - a filled zone reads solidly dark on all three sensors,
// unlike a thin line's edges or a corner/intersection only partially
// covering a sensor's footprint. Tune this against a real reading taken
// sitting on the end zone vs. the darkest ordinary line feature on the
// track.
constexpr float DEEP_BLACK_LEVEL = 0.60f;

int last_heading_left_speed = 10;
int last_heading_right_speed = 100;
int white_gap_left_speed = 110;
int white_gap_right_speed = 100;
bool white_gap_active = false;

constexpr float KP = 65.0f;
constexpr float KD = 7.0f;
constexpr float MAX_CORRECTION = 70.0f;
constexpr float CENTER_DEADBAND = 0.03f;
constexpr float MIDDLE_CENTER_LEVEL = 0.20f;
constexpr float OUTER_CENTER_MAX = 0.08f;

// Per-sensor calibration values measured on this robot.
// constexpr int LEFT_BLACK_THRESHOLD = 75;
// constexpr int MIDDLE_BLACK_THRESHOLD = 45;
// constexpr int RIGHT_BLACK_THRESHOLD = 70;

constexpr int LEFT_BLACK_THRESHOLD = 30;
constexpr int MIDDLE_BLACK_THRESHOLD = 30;
constexpr int RIGHT_BLACK_THRESHOLD = 30;

constexpr int LEFT_BLACK_MAX = 550;
constexpr int MIDDLE_BLACK_MAX = 110;
constexpr int RIGHT_BLACK_MAX = 180;
constexpr int MAX_PWM = 250;
// constexpr int MAX_DRIVE_PWM = 160; // Increase gradually after tuning
constexpr int DEADBAND = 0;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;

const int turning_speed = 100;
const int straight_speed = 100;

const int left_ir = 32;
const int middle_ir = 35;
const int right_ir = 34;
const int left_motorA = 19;
const int left_motorB = 18;
const int right_motorA = 17;
const int right_motorB = 5;
const int left_pwm = 21;
const int right_pwm = 16;
bool capacitor_zone = false;

// End-zone detection: the end zone is just a solid black area the robot
// drives into and stays on. If all three sensors read black continuously for
// long enough, that's it - nothing else on the normal track (corners,
// intersections, dashed-line gaps) stays solid black for anywhere near this
// long.
//
// The dropout tolerance exists only to absorb a single noisy ADC sample
// (a few ms at most - this loop has no blocking delays), not to bridge real
// gaps. Keep it small: a large tolerance lets a track feature that flickers
// in and out of "all three black" (an intersection, a corner the aggressive
// PD oscillates across) stitch many short bursts together into a run that
// never resets, reading as sustained black even though it never truly was -
// this is what caused a run of false end-zone detections.
constexpr uint32_t END_ZONE_BLACK_CONFIRM_MS = 500;
constexpr uint32_t END_ZONE_BLACK_DROPOUT_TOLERANCE_MS = 30;

bool in_end_zone = false;
uint32_t black_run_started_ms = 0; // 0 = no run currently in progress
uint32_t last_all_black_ms = 0;

// Once a ball has been captured, never look for the end zone again for the
// rest of the run - regardless of what the sensors see afterward.
bool ball_retrieved = false;

// After retrieval, the same sustained-deep-black signature reappears once
// when the robot makes it back to the (also solid black) start box. Tracked
// independently from the outbound run's state above so the two can never be
// confused with each other.
uint32_t start_box_run_started_ms = 0;
uint32_t last_start_box_black_ms = 0;

// Detecting the start box only proves the sensors (near the front of the
// chassis) have reached it - the rest of the robot hasn't necessarily
// caught up yet, and the rules require the full body to stop inside. Rather
// than guess from sensor geometry, just keep driving straight for a fixed,
// hand-tuned duration before stopping for good. Tune this on the assembled
// robot against the actual box size.
constexpr uint32_t DRIVE_INTO_START_BOX_MS = 2550;
bool driving_into_start_box = false;
uint32_t driving_into_start_box_started_ms = 0;
bool finished = false; // Latched once stopped in the start box; attempt over.

// The start box is also a solid black square (per the ruleset, just smaller
// than the end box), and the robot begins the run sitting in it - so a raw
// sustained-black check alone can't tell the two apart. The track otherwise
// only has thin line, corners, and intersections between them, so "has the
// robot seen a genuine white gap yet" is a reliable, hardware/size-agnostic
// proxy for "has it actually left the start box's vicinity." End-zone
// detection stays fully disarmed until this is true.
bool has_seen_white_gap = false;

// Once ball retrieval reports done (successful capture or not - that
// distinction isn't made yet), drive straight out. The robot starts this
// still standing on the solid black end zone, so it must first see genuine
// white (i.e. actually leave the black square) before it starts watching for
// black again - otherwise it re-triggers the end-zone detector immediately
// off the ground it's already standing on. Only once clear of the black does
// finding a line again mean the moat ringing the end zone, not the zone
// itself.
bool exiting_end_zone = false;
bool exit_cleared_black_zone = false;

// Drive straight out of the start box for a fixed initial period before
// handing off to normal line following - avoids reacting to whatever the
// sensors see while still settling/leaving the box at the very start of a
// run, before there's a line to actually follow yet.
constexpr uint32_t INITIAL_STRAIGHT_DRIVE_MS = 1000;
uint32_t start_ms = 0;

int determine_drive_mode();
void pid_drive();
int calculate_pid_speed(int sensor_pin);
void drive_motors(int left_vel, int right_vel);
bool check_black(int sensor_pin);
int black_threshold_for_level(int sensor_pin, float level);
int black_threshold_for(int sensor_pin);
int apply_minimum_triple_pwm(int velocity);
void end_zone();
void stop();
bool update_end_zone_detector(bool all_deep_black);
bool detect_sustained_black(bool all_deep_black, uint32_t &run_started_ms,
                             uint32_t &last_seen_ms);

void setup()
{
  Serial.begin(115200);

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

  start_ms = millis();
}

void loop()
{
  if (finished)
  {
    // Attempt is over - stay stopped forever.
    return;
  }

  if (millis() - start_ms < INITIAL_STRAIGHT_DRIVE_MS)
  {
    drive_motors(straight_speed, straight_speed);
    return;
  }

  if (driving_into_start_box)
  {
    if (millis() - driving_into_start_box_started_ms < DRIVE_INTO_START_BOX_MS)
    {
      drive_motors(straight_speed, straight_speed);
      return;
    }

    Serial.println("[StartZone] drive-in complete -> stopping, attempt finished");
    stop();
    finished = true;
    return;
  }

  if (exiting_end_zone)
  {
    const int left_value = analogRead(left_ir);
    const int middle_value = analogRead(middle_ir);
    const int right_value = analogRead(right_ir);

    if (!exit_cleared_black_zone)
    {
      const bool all_very_white =
          left_value < LEFT_BLACK_THRESHOLD &&
          middle_value < MIDDLE_BLACK_THRESHOLD &&
          right_value < RIGHT_BLACK_THRESHOLD;

      if (all_very_white)
      {
        Serial.println("[EndZone] cleared black zone -> now looking for line");
        exit_cleared_black_zone = true;
      }

      drive_motors(straight_speed, straight_speed);
      return;
    }

    const bool left_black = left_value > black_threshold_for(left_ir);
    const bool middle_black = middle_value > black_threshold_for(middle_ir);
    const bool right_black = right_value > black_threshold_for(right_ir);

    if (left_black || middle_black || right_black)
    {
      // Found the moat line ringing the end zone - hand control back to
      // normal line following.
      Serial.println("[EndZone] line found -> resuming line following");
      exiting_end_zone = false;
      exit_cleared_black_zone = false;
      in_end_zone = false;
      black_run_started_ms = 0;
      return;
    }

    drive_motors(straight_speed, straight_speed);
    return;
  }

  if (in_end_zone)
  {
    const BallRetrieveCommand command = update_ball_retrieval();

    if (command.capture_complete)
    {
      Serial.println("[EndZone] retrieval complete -> driving out to find line");
      ball_retrieved = true;
      exiting_end_zone = true;
      return;
    }

    drive_motors(command.left_speed, command.right_speed);
    return;
  }

  determine_drive_mode();
}

int determine_drive_mode()
{
  const int left_value = analogRead(left_ir);
  const int middle_value = analogRead(middle_ir);
  const int right_value = analogRead(right_ir);

  const bool left_black = left_value > black_threshold_for(left_ir);
  const bool middle_black = middle_value > black_threshold_for(middle_ir);
  const bool right_black = right_value > black_threshold_for(right_ir);

  const bool all_very_white =
      left_value < LEFT_BLACK_THRESHOLD &&
      middle_value < MIDDLE_BLACK_THRESHOLD &&
      right_value < RIGHT_BLACK_THRESHOLD;

  if (all_very_white && !has_seen_white_gap)
  {
    Serial.println("[EndZone] first white gap seen -> end-zone detection armed");
    has_seen_white_gap = true;
  }

  const bool all_deep_black =
      left_value > black_threshold_for_level(left_ir, DEEP_BLACK_LEVEL) &&
      middle_value > black_threshold_for_level(middle_ir, DEEP_BLACK_LEVEL) &&
      right_value > black_threshold_for_level(right_ir, DEEP_BLACK_LEVEL);

  // This only takes control once the sensors have read genuinely, deeply
  // black (not merely past the lenient line-following threshold) for the
  // full confirm duration - and only once the robot has seen a genuine white
  // gap at least once. The start box is also a solid black square the robot
  // begins the run standing in, and without that gate this would trigger on
  // it immediately, before ever following any line. Once a ball has been
  // retrieved, this is skipped permanently - the end zone should never be
  // looked for again for the rest of the run.
  if (has_seen_white_gap && !ball_retrieved && update_end_zone_detector(all_deep_black))
  {
    return 8;
  }

  // Once retrieval is done, the only other solid black region left on the
  // course should be the start box the robot began in - watch for the same
  // signature reappearing and, once confirmed, drive in a bit further and
  // stop for good.
  if (ball_retrieved &&
      detect_sustained_black(all_deep_black, start_box_run_started_ms,
                              last_start_box_black_ms))
  {
    Serial.println("[StartZone] sustained deep black confirmed -> driving in to stop");
    driving_into_start_box = true;
    driving_into_start_box_started_ms = millis();
    return 9;
  }

  const int black_sensor_count =
      static_cast<int>(left_black) +
      static_cast<int>(middle_black) +
      static_cast<int>(right_black);

  if (black_sensor_count == 3 && previous_black_sensor_count != 3)
  {
    triple_black_active = true;
    triple_black_is_corner = (previous_black_sensor_count == 2);
  }
  else if (black_sensor_count != 3)
  {
    triple_black_active = false;
  }

  previous_black_sensor_count = black_sensor_count;

  if (!all_very_white)
  {
    white_gap_active = false;
  }

  // Triple black: preserve the previous PD heading, but travel slowly
  // until the robot leaves this wide black region.
  if (triple_black_active)
  {
    // Entering triple black from zero or one black sensor is treated as an
    // obstacle: continue straight rather than initiating a corner turn.
    if (!triple_black_is_corner)
    {
      drive_motors(straight_speed, straight_speed);
      return 7;
    }

    // Separate the previous heading into forward motion and turn amount.
    const float heading_forward =
        (last_heading_left_speed + last_heading_right_speed) / 2.0f;

    const float heading_turn =
        (last_heading_left_speed - last_heading_right_speed) / 2.0f;

    // Slow forward travel, but preserve/amplify the steering difference.
    const int slow_left_speed = constrain(
        static_cast<int>(
            (heading_forward * TRIPLE_BLACK_SPEED_SCALE) +
            (heading_turn * TRIPLE_TURN_SCALE)),
        -MAX_PWM,
        MAX_PWM);

    const int slow_right_speed = constrain(
        static_cast<int>(
            (heading_forward * TRIPLE_BLACK_SPEED_SCALE) -
            (heading_turn * TRIPLE_TURN_SCALE)),
        -MAX_PWM,
        MAX_PWM);

    drive_motors(
        apply_minimum_triple_pwm(slow_left_speed),
        apply_minimum_triple_pwm(slow_right_speed));
    return 6;
  }

  // One or two sensors on black: return to normal PD steering.
  if (left_black || middle_black || right_black)
  {
    pid_drive();
    return 1;
  }

  // A faint reading is not a full white gap, so resume normal steering rather
  // than preserving the old slow command.
  if (!all_very_white)
  {
    pid_drive();
    return 2;
  }

  // Capture the last heading once on entry to a genuinely white gap. Replaying
  // the same scaled pair preserves the curve without reducing it every loop.
  if (!white_gap_active)
  {
    // white_gap_left_speed = last_left_speed;
    // white_gap_right_speed = last_right_speed;
    white_gap_active = true;
  }

  drive_motors(
      static_cast<int>(white_gap_left_speed * WHITE_GAP_SPEED_SCALE),
      static_cast<int>(white_gap_right_speed * WHITE_GAP_SPEED_SCALE));
  return 0;
}

void pid_drive()
{
  capacitor_zone = false;

  static float previous_error = 0.0f;
  static float filtered_derivative = 0.0f;
  static unsigned long previous_time_us = 0;

  const unsigned long now_us = micros();

  float dt = (previous_time_us == 0)
                 ? 0.01f
                 : (now_us - previous_time_us) / 1000000.0f;

  previous_time_us = now_us;
  dt = constrain(dt, 0.002f, 0.05f);

  const int left_value = analogRead(left_ir);
  const int middle_value = analogRead(middle_ir);
  const int right_value = analogRead(right_ir);

  const float left_black = constrain(
      (left_value - LEFT_BLACK_THRESHOLD) /
          static_cast<float>(LEFT_BLACK_MAX - LEFT_BLACK_THRESHOLD),
      0.0f,
      1.0f);

  const float middle_black = constrain(
      (middle_value - MIDDLE_BLACK_THRESHOLD) /
          static_cast<float>(MIDDLE_BLACK_MAX - MIDDLE_BLACK_THRESHOLD),
      0.0f,
      1.0f);

  const float right_black = constrain(
      (right_value - RIGHT_BLACK_THRESHOLD) /
          static_cast<float>(RIGHT_BLACK_MAX - RIGHT_BLACK_THRESHOLD),
      0.0f,
      1.0f);

  const uint32_t now_ms = millis();
  const bool long_time_since_line =
      (now_ms - last_genuine_line_ms) > LINE_REACQUIRE_TIMEOUT_MS;

  if (left_black > 0.0f || middle_black > 0.0f || right_black > 0.0f)
  {
    last_genuine_line_ms = now_ms;
  }

  const bool lone_left_hit =
      left_black >= PIVOT_BLACK_LEVEL && right_black <= PIVOT_OTHER_SIDE_MAX;
  const bool lone_right_hit =
      right_black >= PIVOT_BLACK_LEVEL && left_black <= PIVOT_OTHER_SIDE_MAX;

  if (!ignoring_lone_outer_hit && long_time_since_line &&
      (lone_left_hit || lone_right_hit))
  {
    Serial.println("[LineFollow] lone outer hit after long white -> ignoring, reacquiring line");
    ignoring_lone_outer_hit = true;
  }

  if (ignoring_lone_outer_hit)
  {
    const bool second_sensor_confirms =
        middle_black > 0.0f || (left_black > 0.0f && right_black > 0.0f);

    if (!second_sensor_confirms)
    {
      drive_motors(straight_speed, straight_speed);
      return;
    }

    Serial.println("[LineFollow] second sensor confirmed -> resuming regular driving");
    ignoring_lone_outer_hit = false;
  }

  // Preserve pivoting only for an obvious sharp corner.
  if (lone_left_hit)
  {
    drive_motors(-turning_speed / 2, turning_speed / 2);
    return;
  }

  if (lone_right_hit)
  {
    drive_motors(turning_speed / 2, -turning_speed / 2);
    return;
  }

  // Use the outer sensors for steering. The middle sensor confirms the line
  // is present, but should not dilute a strong left/right corner correction.
  const float outer_black = left_black + right_black;

  // The line is confidently centered only when the middle sees it strongly
  // and the outer sensors see, at most, a small overlap of the line.
  const bool centered_on_middle =
      middle_black >= MIDDLE_CENTER_LEVEL &&
      left_black <= OUTER_CENTER_MAX &&
      right_black <= OUTER_CENTER_MAX;

  float error = 0.0f;

  if (!centered_on_middle && outer_black > 0.01f)
  {
    // -1 = strongly left, 0 = balanced, +1 = strongly right.
    error = (right_black - left_black) / outer_black;
  }

  // Ignore tiny sensor-noise corrections near the center.
  if (fabsf(error) < CENTER_DEADBAND)
  {
    error = 0.0f;
  }

  const float raw_derivative = constrain(
      (error - previous_error) / dt,
      -4.0f,
      4.0f);

  // Smooth derivative so sensor noise does not cause rapid steering changes.
  filtered_derivative =
      (0.75f * filtered_derivative) + (0.25f * raw_derivative);

  const float correction = constrain(
      (KP * error) + (KD * filtered_derivative),
      -MAX_CORRECTION,
      MAX_CORRECTION);

  previous_error = error;

  const int left_speed = constrain(
      static_cast<int>(straight_speed + correction),
      -MAX_PWM,
      MAX_PWM);

  const int right_speed = constrain(
      static_cast<int>(straight_speed - correction),
      -MAX_PWM,
      MAX_PWM);

  // Save the normal PD heading before issuing the motor command.
  last_heading_left_speed = left_speed;
  last_heading_right_speed = right_speed;

  drive_motors(left_speed, right_speed);
}

// int calculate_pid_speed(int sensor_pin)
// {
//   const int sensor_value = analogRead(sensor_pin);

//   // Always use valid, consistent PWM limits.
//   const int cruise_speed = constrain(straight_speed, 0, MAX_PWM);
//   const int max_speed = constrain(MAX_DRIVE_PWM, cruise_speed, MAX_PWM);

//   const int error = sensor_value - BLACK_THRESHOLD;

//   // White or near-black threshold: drive at the fixed cruise speed.
//   if (error <= DEADBAND)
//   {
//     return cruise_speed;
//   }

//   const int limited_error = constrain(
//       error,
//       0,
//       BLACK_MAX_VALUE - BLACK_THRESHOLD);

//   return map(
//       limited_error,
//       0,
//       BLACK_MAX_VALUE - BLACK_THRESHOLD,
//       cruise_speed,
//       max_speed);
// }

void set_motor(int pwm_pin, int direction_pin_1, int direction_pin_2, int velocity)
{
  velocity = constrain(velocity, -MAX_PWM, MAX_PWM);

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
      (pwm_pin == left_pwm) ? LEFT_PWM_CHANNEL : RIGHT_PWM_CHANNEL;
  ledcWrite(pwm_channel, pwm);
}

void drive_motors(int left_vel, int right_vel)
{
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

int black_threshold_for_level(int sensor_pin, float level)
{
  if (sensor_pin == left_ir)
  {
    return LEFT_BLACK_THRESHOLD + static_cast<int>(
        (LEFT_BLACK_MAX - LEFT_BLACK_THRESHOLD) * level);
  }

  if (sensor_pin == middle_ir)
  {
    return MIDDLE_BLACK_THRESHOLD + static_cast<int>(
        (MIDDLE_BLACK_MAX - MIDDLE_BLACK_THRESHOLD) * level);
  }

  return RIGHT_BLACK_THRESHOLD + static_cast<int>(
      (RIGHT_BLACK_MAX - RIGHT_BLACK_THRESHOLD) * level);
}

int black_threshold_for(int sensor_pin)
{
  return black_threshold_for_level(sensor_pin, DETECT_BLACK_LEVEL);
}

int apply_minimum_triple_pwm(int velocity)
{
  if (velocity > 0 && velocity < MIN_TRIPLE_MOVING_PWM)
  {
    return MIN_TRIPLE_MOVING_PWM;
  }

  if (velocity < 0 && velocity > -MIN_TRIPLE_MOVING_PWM)
  {
    return -MIN_TRIPLE_MOVING_PWM;
  }

  return velocity;
}

void end_zone()
{
  capacitor_zone = false; // Reset capacitor zone flag
  stop();                 // Stop the motors
  begin_ball_retrieval();
}

// Has `all_deep_black` been continuously true (tolerating a brief noisy
// dropout) for END_ZONE_BLACK_CONFIRM_MS? Callers each supply their own
// run-state pair so independent detections (the end zone, later the start
// box) never interfere with each other.
bool detect_sustained_black(bool all_deep_black, uint32_t &run_started_ms,
                             uint32_t &last_seen_ms)
{
  const uint32_t now_ms = millis();

  if (all_deep_black)
  {
    if (run_started_ms == 0)
    {
      run_started_ms = now_ms;
    }
    last_seen_ms = now_ms;

    return now_ms - run_started_ms >= END_ZONE_BLACK_CONFIRM_MS;
  }

  if (run_started_ms != 0 &&
      now_ms - last_seen_ms > END_ZONE_BLACK_DROPOUT_TOLERANCE_MS)
  {
    // The run genuinely ended (not just a momentary dropout) before reaching
    // the confirm duration.
    run_started_ms = 0;
  }

  return false;
}

bool update_end_zone_detector(bool all_deep_black)
{
  if (in_end_zone)
  {
    // Latch the state: line following must not resume during ball capture.
    return true;
  }

  if (detect_sustained_black(all_deep_black, black_run_started_ms,
                              last_all_black_ms))
  {
    Serial.println("[EndZone] sustained deep black confirmed -> end zone");
    in_end_zone = true;
    end_zone();
    return true;
  }

  return false;
}

void stop()
{
  drive_motors(0, 0); // Stop the motors
}
