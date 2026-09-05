#include <Arduino.h>

// Pivot only when one side sees a very strong line and the other is mostly clear.
constexpr float PIVOT_BLACK_LEVEL = 0.55f;
constexpr float PIVOT_OTHER_SIDE_MAX = 0.10f;

int last_left_speed = 0;
int last_right_speed = 0;

// Per-sensor calibration values measured on this robot.
// constexpr int LEFT_BLACK_THRESHOLD = 75;
// constexpr int MIDDLE_BLACK_THRESHOLD = 45;
// constexpr int RIGHT_BLACK_THRESHOLD = 70;

constexpr int LEFT_BLACK_THRESHOLD = 40;
constexpr int MIDDLE_BLACK_THRESHOLD = 40;
constexpr int RIGHT_BLACK_THRESHOLD = 40;

constexpr int LEFT_BLACK_MAX = 2490;
constexpr int MIDDLE_BLACK_MAX = 1810;
constexpr int RIGHT_BLACK_MAX = 1620;
constexpr int MAX_PWM = 160;
constexpr int MAX_DRIVE_PWM = 160; // Increase gradually after tuning
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

  // Any visible line: PID decides between smooth steering and pivot override.
  if (left_black || middle_black || right_black)
  {
    pid_drive();
    return 1;
  }

  // No visible line: explicitly keep the most recent motor command.
  if (capacitor_zone)
  {
    drive_motors(straight_speed, straight_speed);

    while (!check_black(left_ir) &&
           !check_black(middle_ir) &&
           !check_black(right_ir))
    {
      delay(10);
    }

    stop();
    return 5;
  }

  drive_motors(last_left_speed, last_right_speed);
  return 0;
}

void pid_drive()
{
  capacitor_zone = false;

  const int left_value = analogRead(left_ir);
  const int right_value = analogRead(right_ir);

  const int left_sensor_range =
      LEFT_BLACK_MAX - LEFT_BLACK_THRESHOLD;

  const int right_sensor_range =
      RIGHT_BLACK_MAX - RIGHT_BLACK_THRESHOLD;

  const float left_black = constrain(
      (left_value - LEFT_BLACK_THRESHOLD) /
          static_cast<float>(left_sensor_range),
      0.0f,
      1.0f);

  const float right_black = constrain(
      (right_value - RIGHT_BLACK_THRESHOLD) /
          static_cast<float>(right_sensor_range),
      0.0f,
      1.0f);

  // Very strong black on only one side: pivot on the spot.
  if (left_black >= PIVOT_BLACK_LEVEL &&
      right_black <= PIVOT_OTHER_SIDE_MAX)
  {
    drive_motors(-turning_speed, turning_speed);
    return;
  }

  if (right_black >= PIVOT_BLACK_LEVEL &&
      left_black <= PIVOT_OTHER_SIDE_MAX)
  {
    drive_motors(turning_speed, -turning_speed);
    return;
  }

  // Normal proportional/PID-style steering.
  constexpr int MAX_CORRECTION = 80;

  const int correction = static_cast<int>(
      (right_black - left_black) * MAX_CORRECTION);

  const int left_speed = constrain(
      straight_speed + correction,
      -MAX_PWM,
      MAX_PWM);

  const int right_speed = constrain(
      straight_speed - correction,
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
