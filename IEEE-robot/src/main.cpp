#include <Arduino.h>

// Recalibrate after changing the sensor mount, tape, or lighting.
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

// Starting points for on-track tuning. Steering remains P-only, using the
// same normalized outer-sensor difference as the previous controller.
constexpr float KP = 80.0f;
constexpr int STRAIGHT_SPEED = 100;
constexpr int CORNER_SPEED = 70;
constexpr unsigned long CONTROL_PERIOD_MS = 10;
constexpr float SENSOR_FILTER_ALPHA = 0.4f;
constexpr int PWM_STEP = 8;                    // Maximum change per wheel per control tick.
constexpr float VISIBILITY_HYSTERESIS = 0.10f; // Fraction of each sensor range.
constexpr int REACQUIRE_TICKS = 3;
constexpr float DIRECTION_MEMORY_ERROR = 0.10f;

// Short gaps get slow forward travel; longer losses trigger a bounded search.
constexpr unsigned long GAP_MS = 120;
constexpr unsigned long FIRST_SEARCH_MS = 600;
constexpr unsigned long SEARCH_TIMEOUT_MS = 2200; // Total time since loss.
constexpr int GAP_SPEED = 60;
constexpr int GAP_MAX_CORRECTION = 20;
constexpr int SEARCH_SPEED = 70;

const int left_ir = 32;
const int middle_ir = 35;
const int right_ir = 34;
const int left_motorA = 19;
const int left_motorB = 18;
const int right_motorA = 17;
const int right_motorB = 5;
const int left_pwm = 21;
const int right_pwm = 16;

// Correct physical motor polarity only here: positive means forward upstream.
constexpr int LEFT_MOTOR_DIRECTION = 1;
constexpr int RIGHT_MOTOR_DIRECTION = 1;

enum class DriveMode
{
  TRACK,
  GAP,
  SEARCH,
  STOPPED
};
DriveMode drive_mode = DriveMode::STOPPED;
int last_left_speed = 0;
int last_right_speed = 0;
int8_t last_line_side = 1; // Default search right if no side has been observed.
float last_error = 0.0f;
float filtered_black[3] = {};
bool filters_initialized = false;
bool line_visible = false;
int reacquire_ticks = 0;
unsigned long last_control_ms = 0;
unsigned long line_lost_since_ms = 0;

void drive_motors(int left_vel, int right_vel);
void stop();

struct SensorSnapshot
{
  bool visible;
  float error;
};

SensorSnapshot read_sensors()
{
  const int pins[3] = {left_ir, middle_ir, right_ir};
  const int thresholds[3] = {LEFT_BLACK_THRESHOLD, MIDDLE_BLACK_THRESHOLD,
                             RIGHT_BLACK_THRESHOLD};
  const int maxima[3] = {LEFT_BLACK_MAX, MIDDLE_BLACK_MAX, RIGHT_BLACK_MAX};
  bool visible = false;
  for (int i = 0; i < 3; ++i)
  {
    const int raw = analogRead(pins[i]); // One reading per sensor per tick.
    const float range = static_cast<float>(maxima[i] - thresholds[i]);
    const float black = constrain((raw - thresholds[i]) / range, 0.0f, 1.0f);
    filtered_black[i] = filters_initialized
                            ? filtered_black[i] + SENSOR_FILTER_ALPHA * (black - filtered_black[i])
                            : black;
    // Use the raw snapshot for visibility so filter lag cannot extend blind
    // driving. A lower exit threshold prevents flicker near the black boundary.
    const float cutoff = thresholds[i] -
                         (line_visible ? VISIBILITY_HYSTERESIS * range : 0.0f);
    visible = visible || raw > cutoff;
  }
  filters_initialized = true;
  line_visible = visible;
  return {visible, filtered_black[2] - filtered_black[0]};
}

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
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(left_pwm, LEFT_PWM_CHANNEL);
  ledcAttachPin(right_pwm, RIGHT_PWM_CHANNEL);
  Serial.begin(9600);
  stop(); // Stay still at startup until the line is confirmed.
  last_control_ms = millis();
}

void loop()
{
  const unsigned long now = millis();
  if (now - last_control_ms < CONTROL_PERIOD_MS)
    return;
  last_control_ms = now; // No burst of catch-up motor updates if a tick is late.
  const SensorSnapshot sensors = read_sensors();

  if (drive_mode == DriveMode::TRACK && !sensors.visible)
  {
    line_lost_since_ms = now;
    drive_mode = DriveMode::GAP;
    reacquire_ticks = 0;
  }

  if (drive_mode != DriveMode::TRACK)
  {
    reacquire_ticks = sensors.visible ? reacquire_ticks + 1 : 0;
    if (reacquire_ticks >= REACQUIRE_TICKS)
    {
      drive_mode = DriveMode::TRACK;
      reacquire_ticks = 0;
    }
  }

  if (drive_mode == DriveMode::TRACK)
  {
    last_error = sensors.error;
    if (last_error < -DIRECTION_MEMORY_ERROR)
      last_line_side = -1;
    else if (last_error > DIRECTION_MEMORY_ERROR)
      last_line_side = 1;

    const float magnitude = last_error < 0 ? -last_error : last_error;
    const int base_speed = STRAIGHT_SPEED -
                           static_cast<int>((STRAIGHT_SPEED - CORNER_SPEED) * magnitude);
    const int correction = static_cast<int>(KP * last_error);
    // Ordinary tracking never reverses a wheel. Reverse is reserved for search.
    drive_motors(constrain(base_speed + correction, 0, MAX_PWM),
                 constrain(base_speed - correction, 0, MAX_PWM));
    return;
  }

  if (drive_mode == DriveMode::STOPPED)
  {
    stop(); // Can resume if the robot is placed back on the line.
    return;
  }

  const unsigned long blind_ms = now - line_lost_since_ms;
  if (blind_ms >= SEARCH_TIMEOUT_MS)
  {
    drive_mode = DriveMode::STOPPED;
    stop(); // Bypass ramping for the timeout; noisy glimpses don't reset it.
  }
  else if (blind_ms < GAP_MS)
  {
    const int correction = constrain(static_cast<int>(KP * last_error),
                                     -GAP_MAX_CORRECTION, GAP_MAX_CORRECTION);
    drive_motors(GAP_SPEED + correction, GAP_SPEED - correction);
  }
  else
  {
    drive_mode = DriveMode::SEARCH;
    // First turn toward the last seen side, then sweep back the other way.
    const int side = blind_ms < GAP_MS + FIRST_SEARCH_MS
                         ? last_line_side
                         : -last_line_side;
    drive_motors(side * SEARCH_SPEED, -side * SEARCH_SPEED);
  }
}

void set_motor(int pwm_pin, int direction_pin_1, int direction_pin_2, int velocity)
{
  // Everything upstream (steering, gap crossing, search) reasons about
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

// Cross zero for at least one tick before changing motor direction.
int ramp_speed(int current, int target)
{
  target = constrain(target, -MAX_PWM, MAX_PWM);
  if ((current > 0 && target < 0) || (current < 0 && target > 0))
    target = 0;
  return current + constrain(target - current, -PWM_STEP, PWM_STEP);
}

void drive_motors(int left_vel, int right_vel)
{
  last_left_speed = ramp_speed(last_left_speed, left_vel);
  last_right_speed = ramp_speed(last_right_speed, right_vel);
  set_motor(left_pwm, left_motorA, left_motorB, last_left_speed);
  set_motor(right_pwm, right_motorA, right_motorB, last_right_speed);
}

void stop()
{
  last_left_speed = 0;
  last_right_speed = 0;
  set_motor(left_pwm, left_motorA, left_motorB, 0);
  set_motor(right_pwm, right_motorA, right_motorB, 0);
}
