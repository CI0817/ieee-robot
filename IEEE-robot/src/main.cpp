#include <Arduino.h>

constexpr int BLACK_THRESHOLD = 100;  // Ignore readings at or below this
constexpr int BLACK_MAX_VALUE = 2000; // Measure your darkest reading; start with 1000
constexpr int MAX_PWM = 80;
constexpr int MAX_DRIVE_PWM = 80; // Increase gradually after tuning
constexpr int DEADBAND = 0;
const int max_correction = 100;

const int turning_speed = 50;
const int straight_speed = 50;

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

// ultrasonics
// TRIG = 13
// ECHO = 12

// Servo
// SIGNAL = 14

int determine_drive_mode();
void pid_drive();
int calculate_pid_speed(int sensor_pin);
void drive_motors(int left_vel, int right_vel);
bool check_black(int sensor_pin);
void end_zone();
void stop();
void grab_ballz();
int read_USS();

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
  // Serial.begin(9600);
}

void loop()
{
  determine_drive_mode();
}

int determine_drive_mode()
{
  if (check_black(middle_ir))
  {

    if (check_black(left_ir) && check_black(right_ir))
    {
      // If all sensors detect black, check if we are in the end zone
      drive_motors(straight_speed, straight_speed);
      delay(50); // Drive straight a little
      if (check_black(left_ir) && check_black(middle_ir) && check_black(right_ir))
      {
        end_zone();
        return 10; // End zone
      }
      else
      {
        // If not in the end zone, we are in the capacitor zone, drive straight
        capacitor_zone = true;
        return 11; // Found capacitor zone
      }
    }
    else
    {
      // If the middle IR sensor detects black, drive in PID mode
      pid_drive();
      return 1; // PID drive mode
    }
  }

  else if (check_black(left_ir) && check_black(right_ir))
  {
    // If both left and right IR sensors detect black, drive straight
    drive_motors(straight_speed, straight_speed); // Drive straight
    return 2;                                     // Straight drive mode
  }

  else if (check_black(left_ir))
  {
    // If only the left IR sensor detects black, turn left
    pid_drive();
    // drive_motors(-turning_speed, turning_speed); // Turn left
    return 3; // Left turn mode
  }

  else if (check_black(right_ir))
  {
    // If only the right IR sensor detects black, turn right
    pid_drive();
    // drive_motors(turning_speed, -turning_speed); // Turn right
    return 4; // Right turn mode
  }

  else
  { // No sensors detect black
    if (capacitor_zone)
    {
      // If we are in the capacitor zone, drive straight
      drive_motors(straight_speed, straight_speed); // Drive straight
      while (!check_black(left_ir) || !check_black(middle_ir || !check_black(right_ir)))
      {
        delay(10);
      }
      stop();
      return 5; // Straight drive mode in capacitor zone
    }
    else
    {
      // Spin to search for the line
      // drive_motors(turning_speed, -turning_speed); // Spin in place
      // Serial.print("\nSearching for line");
      pid_drive();
      // TODO: add some kind of counter to track when lost. line
      return 0; // Search mode
    }
  }
}

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

  analogWrite(pwm_pin, pwm);
}

void drive_motors(int left_vel, int right_vel)
{
  set_motor(left_pwm, left_motorA, left_motorB, left_vel);
  set_motor(right_pwm, right_motorA, right_motorB, right_vel);
}

bool check_black(int sensor_pin)
{
  int sensor_value = analogRead(sensor_pin);
  return sensor_value > BLACK_THRESHOLD; // Returns true if the sensor detects black
}

void end_zone()
{
  capacitor_zone = false; // Reset capacitor zone flag
  stop();                 // Stop the motors
  drive_motors(turning_speed, -turning_speed);
  delay(1000);
  while (read_USS() > 50)
  {
    drive_motors(turning_speed, -turning_speed);
  }
  while (read_USS() > 5)
  {
    drive_motors(-straight_speed, -straight_speed);
  }
  grab_ballz();

  // Additional logic for end zone can be added here
}

void stop()
{
  drive_motors(0, 0); // Stop the motors
}

void grab_ballz()
{
}

int read_USS()
{
  return 0;
}

void pid_drive()
{
  capacitor_zone = false;

  static float integral = 0.0f;
  static float previous_error = 0.0f;
  static float filtered_derivative = 0.0f;
  static float last_correction = 0.0f;

  static unsigned long previous_time_us = 0;
  static unsigned long lost_line_start_ms = 0;

  constexpr int BASE_SPEED = 40;
  constexpr int TURN_THRESHOLD = 1000;

  constexpr unsigned long DASH_GAP_HOLD_MS = 900;
  constexpr unsigned long RECOVERY_TURN_MS = 600;

  constexpr float KP = 20.0f;
  constexpr float KI = 0.5f;
  constexpr float KD = 2.0f;
  constexpr float INTEGRAL_LIMIT = 0.50f;
  constexpr float MAX_CORRECTION = 35.0f;

  const unsigned long now_us = micros();

  float dt = (previous_time_us == 0)
                 ? 0.01f
                 : (now_us - previous_time_us) / 1000000.0f;

  previous_time_us = now_us;
  dt = constrain(dt, 0.002f, 0.05f);

  const int left_value = analogRead(left_ir);
  const int middle_value = analogRead(middle_ir);
  const int right_value = analogRead(right_ir);

  const bool all_white =
      left_value < BLACK_THRESHOLD &&
      middle_value < BLACK_THRESHOLD &&
      right_value < BLACK_THRESHOLD;

  // Bridge dashed-line gaps using the same curve as before the gap.
  if (all_white)
  {
    if (lost_line_start_ms == 0)
    {
      lost_line_start_ms = millis();
    }

    const unsigned long lost_time_ms = millis() - lost_line_start_ms;

    float continued_correction = last_correction;

    const float correction_magnitude =
        (last_correction >= 0.0f)
            ? last_correction
            : -last_correction;

    // If the robot has not found the line after the expected dash gap,
    // gradually tighten the previous curve to search for it.
    if (lost_time_ms > DASH_GAP_HOLD_MS &&
        correction_magnitude > 1.0f)
    {
      const float recovery_amount = constrain(
          (lost_time_ms - DASH_GAP_HOLD_MS) /
              static_cast<float>(RECOVERY_TURN_MS),
          0.0f,
          1.0f);

      const float direction =
          (last_correction >= 0.0f) ? 1.0f : -1.0f;

      continued_correction = direction * (correction_magnitude +
                                          ((MAX_CORRECTION - correction_magnitude) *
                                           recovery_amount));
    }

    const int left_speed = constrain(
        static_cast<int>(BASE_SPEED - continued_correction),
        0,
        MAX_PWM);

    const int right_speed = constrain(
        static_cast<int>(BASE_SPEED + continued_correction),
        0,
        MAX_PWM);

    drive_motors(left_speed, right_speed);
    return;
  }

  lost_line_start_ms = 0;

  // Sharp left: left strongly black and right white.
  if (left_value >= TURN_THRESHOLD &&
      right_value < BLACK_THRESHOLD)
  {
    integral = 0.0f;
    previous_error = 0.0f;
    filtered_derivative = 0.0f;
    last_correction = -MAX_CORRECTION;

    drive_motors(-turning_speed * 3 / 2, turning_speed / 2);
    return;
  }

  // Sharp right: right strongly black and left white.
  if (right_value >= TURN_THRESHOLD &&
      left_value < BLACK_THRESHOLD)
  {
    integral = 0.0f;
    previous_error = 0.0f;
    filtered_derivative = 0.0f;
    last_correction = MAX_CORRECTION;

    drive_motors(turning_speed / 2, -turning_speed * 3 / 2);
    return;
  }

  const int sensor_range = BLACK_MAX_VALUE - BLACK_THRESHOLD;

  const int left_black = constrain(
      left_value - BLACK_THRESHOLD, 0, sensor_range);

  const int right_black = constrain(
      right_value - BLACK_THRESHOLD, 0, sensor_range);

  const float error =
      (right_black - left_black) / static_cast<float>(sensor_range);

  integral += error * dt;
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  const float raw_derivative = constrain(
      (error - previous_error) / dt,
      -3.0f,
      3.0f);

  filtered_derivative =
      (0.7f * filtered_derivative) + (0.3f * raw_derivative);

  const float correction = constrain(
      (KP * error) + (KI * integral) + (KD * filtered_derivative),
      -MAX_CORRECTION,
      MAX_CORRECTION);

  previous_error = error;
  last_correction = correction;

  const int left_speed = constrain(
      static_cast<int>(BASE_SPEED - correction),
      0,
      MAX_PWM);

  const int right_speed = constrain(
      static_cast<int>(BASE_SPEED + correction),
      0,
      MAX_PWM);

  drive_motors(left_speed, right_speed);
}
