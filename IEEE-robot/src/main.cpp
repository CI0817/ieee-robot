#include <Arduino.h>

// Per-sensor calibration: black readings must be higher than white readings.
constexpr int LEFT_BLACK_THRESHOLD = 45;
constexpr int MIDDLE_BLACK_THRESHOLD = 40;
constexpr int RIGHT_BLACK_THRESHOLD = 45;
constexpr int LEFT_BLACK_MAX = 760;
constexpr int MIDDLE_BLACK_MAX = 165;
constexpr int RIGHT_BLACK_MAX = 270;
constexpr int MAX_PWM = 160;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;

// Starting values: tune on the robot at low speed before increasing cruise PWM.
constexpr int straight_speed = 100;
constexpr int turning_speed = 90;
constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t MIN_TURN_MS = 60;
constexpr uint32_t TURN_TIMEOUT_MS = 1200;
constexpr uint32_t GAP_CROSS_MS = 180;
constexpr int GAP_SPEED = 65;
constexpr uint32_t SEARCH_LEG_MS = 250;
constexpr uint32_t SEARCH_TIMEOUT_MS = 6 * SEARCH_LEG_MS;
constexpr int BLACK_HYSTERESIS = 5;
constexpr int CONFIRM_SAMPLES = 3;
constexpr int LOST_CONFIRM_SAMPLES = 3;
constexpr float CORNER_MIN_LEVEL = 0.12f;
constexpr float STEERING_DEADBAND = 0.03f;
constexpr int MIN_FOLLOW_PWM = 40;
// PD gains use normalized side error; KD has units PWM * seconds / error.
constexpr float STEERING_KP = 55.0f;
constexpr float STEERING_KD = 0.8f;
constexpr float DERIVATIVE_FILTER_SECONDS = 0.02f;
constexpr float MAX_DAMPING_PWM = 20.0f;
constexpr float MAX_STEERING_PWM = 55.0f;

const int left_ir = 32;
const int middle_ir = 35;
const int right_ir = 34;
const int left_motorA = 19;
const int left_motorB = 18;
const int right_motorA = 17;
const int right_motorB = 5;
const int left_pwm = 21;
const int right_pwm = 16;

int last_left_speed = 0;
int last_right_speed = 0;
void drive_motors(int left_vel, int right_vel);

struct Sensor
{
  int pin;
  int threshold;
  int maximum;
  bool black = false;
  float strength = 0.0f;

  Sensor(int sensor_pin, int black_threshold, int black_maximum)
      : pin(sensor_pin), threshold(black_threshold), maximum(black_maximum) {}

  void sample()
  {
    const int raw = analogRead(pin);
    // Separate entry/exit thresholds prevent noise toggling line presence.
    black = raw > (black ? threshold - BLACK_HYSTERESIS : threshold);
    strength = constrain(
        (raw - threshold) / static_cast<float>(maximum - threshold), 0.0f, 1.0f);
  }
};

Sensor left_sensor{left_ir, LEFT_BLACK_THRESHOLD, LEFT_BLACK_MAX};
Sensor middle_sensor{middle_ir, MIDDLE_BLACK_THRESHOLD, MIDDLE_BLACK_MAX};
Sensor right_sensor{right_ir, RIGHT_BLACK_THRESHOLD, RIGHT_BLACK_MAX};

enum class DriveState
{
  Follow,
  Gap,
  Search,
  Turn,
  Stopped
};
DriveState drive_state = DriveState::Follow;
uint32_t last_control_ms = 0;
uint32_t turn_started_ms = 0;
uint32_t gap_started_ms = 0;
int visible_samples = 0;
int lost_samples = 0;
int last_direction = 0; // -1 = left, +1 = right
int turn_direction = 0;
int corner_direction = 0;
int corner_samples = 0;
int centered_samples = 0;
float steering = 0.0f;
float previous_error = 0.0f;
float filtered_derivative = 0.0f;
bool steering_ready = false;

void reset_steering()
{
  steering = 0.0f;
  previous_error = 0.0f;
  filtered_derivative = 0.0f;
  steering_ready = false;
}

float calculate_steering(float difference, bool centered, float dt)
{
  // No old correction should keep turning the robot after it reaches center.
  if (centered)
  {
    reset_steering();
    return 0.0f;
  }
  // Continuous deadband: no jump in output at the edge of the neutral region.
  const float error = difference > STEERING_DEADBAND ? difference - STEERING_DEADBAND
      : difference < -STEERING_DEADBAND ? difference + STEERING_DEADBAND : 0.0f;
  if (steering_ready && dt > 0.0f && dt <= 0.05f)
  {
    const float derivative = (error - previous_error) / dt;
    const float alpha = dt / (DERIVATIVE_FILTER_SECONDS + dt);
    filtered_derivative += alpha * (derivative - filtered_derivative);
  }
  else
    filtered_derivative = 0.0f;
  previous_error = error;
  steering_ready = true;
  const float damping = constrain(STEERING_KD * filtered_derivative,
                                  -MAX_DAMPING_PWM, MAX_DAMPING_PWM);
  // Filter only the derivative. Delaying the whole correction continued to
  // steer in the old direction after the robot had crossed the line.
  return constrain(STEERING_KP * error + damping, -MAX_STEERING_PWM, MAX_STEERING_PWM);
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
  drive_motors(0, 0);
}

void begin_turn(int direction, uint32_t now)
{
  drive_state = DriveState::Turn;
  turn_direction = direction;
  turn_started_ms = now;
  centered_samples = 0;
  corner_samples = 0;
  lost_samples = 0;
  reset_steering();
}

void loop()
{
  const uint32_t now = millis();
  if (now - last_control_ms < CONTROL_PERIOD_MS)
    return;
  const float dt = (now - last_control_ms) / 1000.0f;
  last_control_ms = now;

  // One shared sensor snapshot per control tick, including the middle sensor.
  left_sensor.sample();
  middle_sensor.sample();
  right_sensor.sample();
  const bool left = left_sensor.black;
  const bool middle = middle_sensor.black;
  const bool right = right_sensor.black;

  // Keep sensing after a timeout; replacing the robot on tape resumes it.
  if (drive_state == DriveState::Stopped)
  {
    visible_samples = left || middle || right ? visible_samples + 1 : 0;
    if (visible_samples < CONFIRM_SAMPLES)
    {
      drive_motors(0, 0);
      return;
    }
    drive_state = DriveState::Follow;
    visible_samples = 0;
    corner_samples = 0;
    corner_direction = 0;
    last_direction = 0;
    lost_samples = 0;
    reset_steering();
  }

  if (drive_state == DriveState::Gap)
  {
    if (left || middle || right)
      drive_state = DriveState::Follow;
    else if (now - gap_started_ms >= GAP_CROSS_MS)
    {
      begin_turn(1, now); // No side evidence: sweep right, left, then right.
      drive_state = DriveState::Search;
    }
    else
    {
      drive_motors(GAP_SPEED, GAP_SPEED);
      return;
    }
  }

  if (drive_state == DriveState::Follow)
  {
    if (middle)
      last_direction = 0;

    // Require meaningful side strength, not just a reading above white noise.
    const bool strong_side = left ? left_sensor.strength >= CORNER_MIN_LEVEL
                                  : right_sensor.strength >= CORNER_MIN_LEVEL;
    const int candidate = !middle && (left != right) && strong_side ? (left ? -1 : 1) : 0;
    corner_samples = candidate != 0 ? (candidate == corner_direction ? corner_samples + 1 : 1) : 0;
    corner_direction = candidate;
    if (corner_samples >= 2)
      last_direction = candidate;
    lost_samples = left || middle || right ? 0 : lost_samples + 1;
    if (corner_samples >= CONFIRM_SAMPLES)
      begin_turn(candidate, now);
    else if (lost_samples > 0)
    {
      // A brief dropout must not change steering or trigger a recovery pivot.
      if (lost_samples < LOST_CONFIRM_SAMPLES)
      {
        steering_ready = false;
        return;
      }
      // A known side recovers immediately; centered loss may be a short gap.
      if (last_direction != 0)
        begin_turn(last_direction, now);
      else
      {
        drive_state = DriveState::Gap;
        gap_started_ms = now;
        reset_steering();
        drive_motors(GAP_SPEED, GAP_SPEED);
        return;
      }
    }
  }

  if (drive_state == DriveState::Turn || drive_state == DriveState::Search)
  {
    const uint32_t elapsed = now - turn_started_ms;
    const bool searching = drive_state == DriveState::Search;
    if (elapsed >= (searching ? SEARCH_TIMEOUT_MS : TURN_TIMEOUT_MS))
    {
      drive_state = DriveState::Stopped;
      visible_samples = 0;
      drive_motors(0, 0);
      return;
    }
    // Do not release a turn on a single noisy sample or the incoming line.
    // Adjacent sensors may still overlap tape: center visibility is sufficient.
    const bool centered = middle;
    centered_samples = centered && now - turn_started_ms >= MIN_TURN_MS ? centered_samples + 1 : 0;
    if (centered_samples < CONFIRM_SAMPLES)
    {
      // Increasing sweep lengths cross the starting heading in both directions.
      const int direction = searching
                                ? (elapsed < SEARCH_LEG_MS || elapsed >= 3 * SEARCH_LEG_MS ? 1 : -1)
                                : turn_direction;
      drive_motors(direction * turning_speed, -direction * turning_speed);
      return;
    }
    drive_state = DriveState::Follow;
    last_direction = 0;
    corner_direction = 0;
    reset_steering();
  }

  steering = calculate_steering(right_sensor.strength - left_sensor.strength,
                                middle && !left && !right, dt);
  // Keep ordinary following wheels powered; pivots belong to the turn state.
  drive_motors(constrain(static_cast<int>(straight_speed + steering), MIN_FOLLOW_PWM, MAX_PWM),
               constrain(static_cast<int>(straight_speed - steering), MIN_FOLLOW_PWM, MAX_PWM));
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
