#include <Arduino.h>

constexpr float TRIPLE_BLACK_LEVEL = 0.70f;

constexpr float TURN_INTENT_ERROR = 0.12f;
constexpr unsigned long TURN_INTENT_WINDOW_MS = 250;

unsigned long last_turn_intent_ms = 0;

bool triple_black_latched = false;

bool is_strong_triple_black();

constexpr int CORNER_PIVOT_SPEED = 130;
constexpr unsigned long MIN_CORNER_PIVOT_MS = 250;
constexpr unsigned long MAX_CORNER_PIVOT_MS = 900;
constexpr float DIRECTION_SAVE_ERROR = 0.08f;

int last_turn_direction = 1; // 1 = right, -1 = left
bool corner_turn_active = false;
unsigned long corner_turn_start_ms = 0;

// Pivot only when one side sees a very strong line and the other is mostly clear.
constexpr float PIVOT_BLACK_LEVEL = 0.90f;
constexpr float PIVOT_OTHER_SIDE_MAX = 0.10f;

int last_left_speed = 0;
int last_right_speed = 0;

constexpr float KP = 55.0f;
constexpr float KD = 7.0f;
constexpr float MAX_CORRECTION = 60.0f;
constexpr float CENTER_DEADBAND = 0.04f;

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

int determine_drive_mode();
void pid_drive();
int calculate_pid_speed(int sensor_pin);
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
  // Serial.begin(9600);
}

void loop()
{
  // drive_motors(50, 50);
  determine_drive_mode();
}

int determine_drive_mode()
{
  const bool left_black = check_black(left_ir);
  const bool middle_black = check_black(middle_ir);
  const bool right_black = check_black(right_ir);

  const bool strong_triple_black = is_strong_triple_black();
  const unsigned long now_ms = millis();

  const bool recent_turn_intent =
    last_turn_intent_ms != 0 &&
    (now_ms - last_turn_intent_ms) <= TURN_INTENT_WINDOW_MS;

  // Rearm the next corner only after this strong triple-black patch clears.
  if (!strong_triple_black)
  {
    triple_black_latched = false;
  }

  // Continue an already-started corner pivot.
  if (corner_turn_active)
  {
    const unsigned long elapsed_ms = now_ms - corner_turn_start_ms;

    const bool centered_on_new_line =
        middle_black && !left_black && !right_black;

    const bool must_keep_pivoting =
        elapsed_ms < MIN_CORNER_PIVOT_MS ||
        (!centered_on_new_line && elapsed_ms < MAX_CORNER_PIVOT_MS);

    if (must_keep_pivoting)
    {
      if (last_turn_direction > 0)
      {
        drive_motors(CORNER_PIVOT_SPEED, -CORNER_PIVOT_SPEED);
      }
      else
      {
        drive_motors(-CORNER_PIVOT_SPEED, CORNER_PIVOT_SPEED);
      }

      return 6;
    }

    corner_turn_active = false;
  }

  // Start one committed pivot only for genuinely strong triple black.
  if (strong_triple_black && !triple_black_latched)
  {
    // Triple black after a recent left/right error: likely a 90-degree turn.
    if (recent_turn_intent)
    {
      triple_black_latched = true;
      corner_turn_active = true;
      corner_turn_start_ms = now_ms;

      if (last_turn_direction > 0)
      {
        drive_motors(CORNER_PIVOT_SPEED, -CORNER_PIVOT_SPEED);
      }
      else
      {
        drive_motors(-CORNER_PIVOT_SPEED, CORNER_PIVOT_SPEED);
      }

      return 6;
    }

    // Triple black while previously travelling straight: treat it as an obstacle.
    drive_motors(last_left_speed, last_right_speed);
    return 7;
  }

  // Normal line following.
  if (left_black || middle_black || right_black)
  {
    pid_drive();
    return 1;
  }

  // Continue the last command through a line gap.
  drive_motors(last_left_speed, last_right_speed);
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

  // Weighted line position:
  // -1 = line under left, 0 = middle, +1 = right.
  const float total_black = left_black + middle_black + right_black;

  float error = 0.0f;

  if (total_black > 0.01f)
  {
    error = (right_black - left_black) / total_black;
  }

  if (error > TURN_INTENT_ERROR)
  {
    last_turn_direction = 1; // right
    last_turn_intent_ms = millis();
  }
  else if (error < -TURN_INTENT_ERROR)
  {
    last_turn_direction = -1; // left
    last_turn_intent_ms = millis();
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
  drive_motors(0, 0); // Stop the motors
}

bool is_strong_triple_black()
{
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

  return left_black >= TRIPLE_BLACK_LEVEL &&
         middle_black >= TRIPLE_BLACK_LEVEL &&
         right_black >= TRIPLE_BLACK_LEVEL;
}