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

// End-zone detection. There's no special marker line - the track's normal
// line simply ends, leaving a stretch of genuine white with nothing beyond
// it but the end zone. So: once the line disappears into real white, watch
// for what comes next. If solid black follows and it sustains (the wide end
// zone, not just a corner glancing across all three sensors), that's it.
// Track resuming as an ordinary 1-2 sensor line (a normal dashed-line gap)
// or nothing showing up before the timeout means it wasn't the end zone.
constexpr uint32_t END_ZONE_GAP_MAX_MS = 600;
constexpr uint32_t END_ZONE_BLACK_CONFIRM_MS = 250;

enum class EndZoneState : uint8_t
{
  FOLLOWING_LINE,
  GAP,
  CONFIRMING_ZONE,
  IN_END_ZONE,
};

EndZoneState end_zone_state = EndZoneState::FOLLOWING_LINE;
uint32_t end_zone_state_started_ms = 0;

int determine_drive_mode();
void pid_drive();
int calculate_pid_speed(int sensor_pin);
void drive_motors(int left_vel, int right_vel);
bool check_black(int sensor_pin);
int black_threshold_for(int sensor_pin);
void end_zone();
void stop();
bool update_end_zone_detector(bool left_black, bool middle_black,
                              bool right_black, bool all_very_white);
const char *end_zone_state_name(EndZoneState state);
void set_end_zone_state(EndZoneState new_state, uint32_t now_ms);

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
  if (end_zone_state == EndZoneState::IN_END_ZONE)
  {
    const BallRetrieveCommand command = update_ball_retrieval();
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

  // This only takes control after the complete marker, gap, and sustained
  // black-zone sequence has been observed. Until then, normal driving below
  // receives exactly the same sensor readings and motor commands as before.
  if (update_end_zone_detector(left_black, middle_black, right_black,
                                all_very_white))
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

const char *end_zone_state_name(EndZoneState state)
{
  switch (state)
  {
  case EndZoneState::FOLLOWING_LINE:
    return "FOLLOWING_LINE";
  case EndZoneState::GAP:
    return "GAP";
  case EndZoneState::CONFIRMING_ZONE:
    return "CONFIRMING_ZONE";
  case EndZoneState::IN_END_ZONE:
    return "IN_END_ZONE";
  }
  return "UNKNOWN";
}

void set_end_zone_state(EndZoneState new_state, uint32_t now_ms)
{
  if (new_state != end_zone_state)
  {
    Serial.print("[EndZone] ");
    Serial.print(end_zone_state_name(end_zone_state));
    Serial.print(" -> ");
    Serial.println(end_zone_state_name(new_state));
  }

  end_zone_state = new_state;
  end_zone_state_started_ms = now_ms;
}

bool update_end_zone_detector(bool left_black, bool middle_black,
                              bool right_black, bool all_very_white)
{
  const bool all_black = left_black && middle_black && right_black;
  const uint32_t now_ms = millis();

  switch (end_zone_state)
  {
  case EndZoneState::FOLLOWING_LINE:
    if (all_very_white)
    {
      // The line has stopped. This might be the gap right before the end
      // zone, or it might just be a normal dashed-line gap - CONFIRMING_ZONE
      // below is what actually tells the two apart.
      set_end_zone_state(EndZoneState::GAP, now_ms);
    }
    break;

  case EndZoneState::GAP:
    if (all_black)
    {
      set_end_zone_state(EndZoneState::CONFIRMING_ZONE, now_ms);
      break;
    }

    // Keep waiting through ambiguous or still-white readings; only give up
    // once nothing resolves into black within the timeout (the line simply
    // resumed as normal 1-2 sensor tracking, i.e. an ordinary gap).
    if (now_ms - end_zone_state_started_ms > END_ZONE_GAP_MAX_MS)
    {
      set_end_zone_state(EndZoneState::FOLLOWING_LINE, now_ms);
    }
    break;

  case EndZoneState::CONFIRMING_ZONE:
    if (!all_black)
    {
      // A short black segment after the gap was not the end zone.
      set_end_zone_state(EndZoneState::FOLLOWING_LINE, now_ms);
      break;
    }

    if (now_ms - end_zone_state_started_ms >= END_ZONE_BLACK_CONFIRM_MS)
    {
      set_end_zone_state(EndZoneState::IN_END_ZONE, now_ms);
      end_zone();
    }
    break;

  case EndZoneState::IN_END_ZONE:
    // Latch the state: line following must not resume during ball capture.
    return true;
  }

  return end_zone_state == EndZoneState::IN_END_ZONE;
}

void stop()
{
  drive_motors(0, 0); // Stop the motors
}
