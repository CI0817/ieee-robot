#include <Arduino.h>

// Per-sensor calibration values measured on this robot.
constexpr int LEFT_BLACK_THRESHOLD = 330;
constexpr int MIDDLE_BLACK_THRESHOLD = 60;
constexpr int RIGHT_BLACK_THRESHOLD = 275;

constexpr int LEFT_BLACK_MAX = 620;
constexpr int MIDDLE_BLACK_MAX = 105;
constexpr int RIGHT_BLACK_MAX = 520;
constexpr int MAX_PWM = 160;
constexpr int MAX_DRIVE_PWM = 160; // Increase gradually after tuning
constexpr int DEADBAND = 0;
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
constexpr int PIVOT_DEBOUNCE_TICKS = 3;

// Safety cutoff: if no sensor has seen black for this long, stop instead of
// coasting on a held command forever (e.g. a stale pivot command with no
// line in sight would otherwise spin in place indefinitely).
constexpr unsigned long MAX_BLIND_MS = 500;

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

// Tracks how long the line has been out of view, for the MAX_BLIND_MS cutoff.
bool line_was_visible = true;
unsigned long line_lost_since_ms = 0;

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
  Serial.begin(9600);
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

  if (left_black || middle_black || right_black)
  {
    // Any sensor seeing black is enough to steer from -- pid_drive() reads
    // the raw analog values itself and decides between smooth steering and
    // the pivot override.
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

  const int left_value = analogRead(left_ir);
  const int right_value = analogRead(right_ir);

  const int left_sensor_range = LEFT_BLACK_MAX - LEFT_BLACK_THRESHOLD;
  const int right_sensor_range = RIGHT_BLACK_MAX - RIGHT_BLACK_THRESHOLD;
  const int max_correction = 80;

  const int left_error = constrain(
      left_value - LEFT_BLACK_THRESHOLD, 0, left_sensor_range);

  const int right_error = constrain(
      right_value - RIGHT_BLACK_THRESHOLD, 0, right_sensor_range);

  const float left_black =
      left_error / static_cast<float>(left_sensor_range);
  const float right_black =
      right_error / static_cast<float>(right_sensor_range);

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
      pivot_state = pivot_candidate_side;
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
    if (pivot_state < 0)
    {
      drive_motors(-turning_speed, 0);
    }
    else
    {
      drive_motors(0, -turning_speed);
    }
    return;
  }

  // Left black -> negative correction -> left slows, right speeds up.
  // Right black -> positive correction -> left speeds up, right slows.
  const int correction = static_cast<int>(
      (right_black - left_black) * max_correction);

  const int left_speed = constrain(
      straight_speed + correction, -MAX_PWM, MAX_PWM);

  const int right_speed = constrain(
      straight_speed - correction, -MAX_PWM, MAX_PWM);

  drive_motors(left_speed, right_speed);
  // Serial.print("\nLeft speed: ");
  // Serial.print(left_speed);
  // Serial.print(" | Right speed: ");
  // Serial.println(right_speed);
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
