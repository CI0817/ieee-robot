#include <Arduino.h>

constexpr int BLACK_THRESHOLD = 100;  // Ignore readings at or below this
constexpr int BLACK_MAX_VALUE = 2000; // Measure your darkest reading; start with 1000
constexpr int MAX_PWM = 80;
constexpr int MAX_DRIVE_PWM = 80; // Increase gradually after tuning
constexpr int DEADBAND = 0;

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

int determine_drive_mode();
void pid_drive();
int calculate_pid_speed(int sensor_pin);
void drive_motors(int left_vel, int right_vel);
bool check_black(int sensor_pin);
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
  Serial.begin(9600);
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
      Serial.print("\nSearching for line");
      pid_drive();
      // TODO: add some kind of counter to track when lost. line
      return 0; // Search mode
    }
  }
}

void pid_drive()
{
  // Serial.print("\nPID Mode");
  capacitor_zone = false;

  const int left_value = analogRead(left_ir);
  const int right_value = analogRead(right_ir);

  // // Strong black: turn in place toward the detected side.
  // if (left_value >= BLACK_MAX_VALUE)
  // {
  //   drive_motors(-turning_speed, turning_speed);
  //   return;
  // }

  // if (right_value >= BLACK_MAX_VALUE)
  // {
  //   drive_motors(turning_speed, -turning_speed);
  //   return;
  // }

  const int sensor_range = BLACK_MAX_VALUE - BLACK_THRESHOLD;
  const int max_correction = 50;

  const int left_error = constrain(
      left_value - BLACK_THRESHOLD, 0, sensor_range);

  const int right_error = constrain(
      right_value - BLACK_THRESHOLD, 0, sensor_range);

  // Left black -> negative correction -> left slows, right speeds up.
  // Right black -> positive correction -> left speeds up, right slows.
  const int correction =
      ((right_error - left_error) * max_correction) / sensor_range;

  const int left_speed = constrain(
      straight_speed + correction, 0, MAX_PWM);

  const int right_speed = constrain(
      straight_speed - correction, 0, MAX_PWM);

  drive_motors(left_speed, right_speed);
  Serial.print("\nLeft speed: ");
  Serial.print(left_speed);
  Serial.print(" | Right speed: ");
  Serial.println(right_speed);
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
  // Additional logic for end zone can be added here
}

void stop()
{
  drive_motors(0, 0); // Stop the motors
}