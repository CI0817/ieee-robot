#include <Arduino.h>
#include <BallRetrieve.h>

// Pivot only when one side sees a very strong line and the other is mostly clear.
constexpr float PIVOT_BLACK_LEVEL = 0.90f;
constexpr float PIVOT_OTHER_SIDE_MAX = 0.10f;

int last_left_speed = 0;
int last_right_speed = 0;

constexpr float WHITE_GAP_SPEED_SCALE = 0.40f;
constexpr int WHITE_GAP_MAX_READING = 35;
// Fraction of each calibrated black range required to count as “detected.”
constexpr float DETECT_BLACK_LEVEL = 0.15f;

int white_gap_left_speed = 0;
int white_gap_right_speed = 0;
bool white_gap_active = false;

constexpr float KP = 65.0f;
constexpr float KD = 7.0f;
constexpr float MAX_CORRECTION = 60.0f;
constexpr float CENTER_DEADBAND = 0.03f;
constexpr float MIDDLE_CENTER_LEVEL = 0.20f;
constexpr float OUTER_CENTER_MAX = 0.08f;

// Per-sensor calibration values measured on this robot.
// constexpr int LEFT_BLACK_THRESHOLD = 75;
// constexpr int MIDDLE_BLACK_THRESHOLD = 45;
// constexpr int RIGHT_BLACK_THRESHOLD = 70;

constexpr int LEFT_BLACK_THRESHOLD = 45;
constexpr int MIDDLE_BLACK_THRESHOLD = 40;
constexpr int RIGHT_BLACK_THRESHOLD = 45;

constexpr int LEFT_BLACK_MAX = 760;
constexpr int MIDDLE_BLACK_MAX = 165;
constexpr int RIGHT_BLACK_MAX = 270;
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
// long. A brief dropout (a seam, a scuff, a wheel bump) within the run
// doesn't reset the clock, since that already once cost us a real detection.
constexpr uint32_t END_ZONE_BLACK_CONFIRM_MS = 500;
constexpr uint32_t END_ZONE_BLACK_DROPOUT_TOLERANCE_MS = 150;

bool in_end_zone = false;
uint32_t black_run_started_ms = 0; // 0 = no run currently in progress
uint32_t last_all_black_ms = 0;

// Once ball retrieval reports done (successful capture or not - that
// distinction isn't made yet), drive straight blind until any sensor finds
// black again. The course has a line ringing the end zone specifically to
// funnel the robot back onto the track this way.
bool exiting_end_zone = false;

int determine_drive_mode();
void pid_drive();
int calculate_pid_speed(int sensor_pin);
void drive_motors(int left_vel, int right_vel);
bool check_black(int sensor_pin);
int black_threshold_for(int sensor_pin);
void end_zone();
void stop();
bool update_end_zone_detector(bool all_black);

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
}

void loop()
{
  if (exiting_end_zone)
  {
    const bool left_black = check_black(left_ir);
    const bool middle_black = check_black(middle_ir);
    const bool right_black = check_black(right_ir);

    if (left_black || middle_black || right_black)
    {
      // Found the moat line ringing the end zone - hand control back to
      // normal line following.
      Serial.println("[EndZone] line found -> resuming line following");
      exiting_end_zone = false;
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
      left_value <= WHITE_GAP_MAX_READING &&
      middle_value <= WHITE_GAP_MAX_READING &&
      right_value <= WHITE_GAP_MAX_READING;

  // This only takes control once the sensors have read solid black for the
  // full confirm duration. Until then, normal driving below receives exactly
  // the same sensor readings and motor commands as before.
  if (update_end_zone_detector(left_black && middle_black && right_black))
  {
    return 8;
  }

  if (!all_very_white)
  {
    white_gap_active = false;
  }

  // Any sensor on black (including all three at once, e.g. a wide corner or
  // an intersection): normal PD steering handles it directly.
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
    white_gap_left_speed = last_left_speed;
    white_gap_right_speed = last_right_speed;
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

  // Preserve pivoting only for an obvious sharp corner.
  if (left_black >= PIVOT_BLACK_LEVEL &&
      right_black <= PIVOT_OTHER_SIDE_MAX)
  {
    drive_motors(-turning_speed / 2, turning_speed / 2);
    return;
  }

  if (right_black >= PIVOT_BLACK_LEVEL &&
      left_black <= PIVOT_OTHER_SIDE_MAX)
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

int black_threshold_for(int sensor_pin)
{
  if (sensor_pin == left_ir)
  {
    return LEFT_BLACK_THRESHOLD + static_cast<int>(
        (LEFT_BLACK_MAX - LEFT_BLACK_THRESHOLD) * DETECT_BLACK_LEVEL);
  }

  if (sensor_pin == middle_ir)
  {
    return MIDDLE_BLACK_THRESHOLD + static_cast<int>(
        (MIDDLE_BLACK_MAX - MIDDLE_BLACK_THRESHOLD) *
        DETECT_BLACK_LEVEL);
  }

  return RIGHT_BLACK_THRESHOLD + static_cast<int>(
      (RIGHT_BLACK_MAX - RIGHT_BLACK_THRESHOLD) *
      DETECT_BLACK_LEVEL);
}

void end_zone()
{
  capacitor_zone = false; // Reset capacitor zone flag
  stop();                 // Stop the motors
  begin_ball_retrieval();
}

bool update_end_zone_detector(bool all_black)
{
  if (in_end_zone)
  {
    // Latch the state: line following must not resume during ball capture.
    return true;
  }

  const uint32_t now_ms = millis();

  if (all_black)
  {
    if (black_run_started_ms == 0)
    {
      black_run_started_ms = now_ms;
      Serial.println("[EndZone] triple-black run started");
    }
    last_all_black_ms = now_ms;

    if (now_ms - black_run_started_ms >= END_ZONE_BLACK_CONFIRM_MS)
    {
      Serial.println("[EndZone] sustained triple-black confirmed -> end zone");
      in_end_zone = true;
      end_zone();
      return true;
    }
  }
  else if (black_run_started_ms != 0 &&
           now_ms - last_all_black_ms > END_ZONE_BLACK_DROPOUT_TOLERANCE_MS)
  {
    // The run genuinely ended (not just a momentary dropout) before reaching
    // the confirm duration - this wasn't the end zone.
    Serial.println("[EndZone] triple-black run ended before confirm threshold");
    black_run_started_ms = 0;
  }

  return false;
}

void stop()
{
  drive_motors(0, 0); // Stop the motors
}
