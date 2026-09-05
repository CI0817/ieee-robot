#include <Arduino.h>
#include <NewPing.h>
#include "BallCapture.h"

static NewPing sonar(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN, 450);

// Channel 2 uses a different timer from main.cpp's motor channels 0 and 1.
constexpr int SERVO_PWM_CHANNEL = 2;
constexpr int SERVO_PWM_BITS = 16;
constexpr int SERVO_PERIOD_US = 20000; // 50 Hz
static int hand_angle = HAND_RAISED_ANGLE;
static bool hand_lowering = false;
static unsigned long last_hand_step_ms = 0;

static void set_hand_angle(int angle)
{
  // Use a conservative 1000–2000 us pulse range; tune positions on the robot.
  const int pulse_us = map(constrain(angle, 0, 180), 0, 180, 1000, 2000);
  const uint32_t duty = static_cast<uint32_t>(pulse_us) * 65535 / SERVO_PERIOD_US;
  ledcWrite(SERVO_PWM_CHANNEL, duty);
}

void setup_capture()
{
  // NewPing configures the ultrasonic pins in its constructor.
  ledcSetup(SERVO_PWM_CHANNEL, 50, SERVO_PWM_BITS);
  ledcAttachPin(CAPTURE_SERVO_PIN, SERVO_PWM_CHANNEL);
  raise_hand();
  delay(500); // Allow time to reach the initial position before checking for a ball.
}

float read_distance_cm()
{
  // Keep successive pings apart, even when called repeatedly.
  delay(60);
  const unsigned int echo_us = sonar.ping();
  const float distance_cm = echo_us / static_cast<float>(US_ROUNDTRIP_CM);
  // NewPing returns 0 for no echo; also reject the sensor's blind region.
  return distance_cm >= 3 && distance_cm <= 450 ? distance_cm : -1.0f;
}

bool ball_detected(float max_distance_cm)
{
  const float distance_cm = read_distance_cm();
  return distance_cm > 0.0f && distance_cm <= max_distance_cm;
}

void lower_hand()
{
  if (hand_lowering || hand_is_lowered())
    return;
  hand_lowering = true;
  last_hand_step_ms = millis();
}

void update_hand()
{
  const unsigned long now = millis();
  if (!hand_lowering || now - last_hand_step_ms < HAND_STEP_INTERVAL_MS)
    return;

  last_hand_step_ms = now;
  // One small step per update, even if the loop was delayed.
  hand_angle += hand_angle < HAND_LOWERED_ANGLE ? 1 : -1;
  set_hand_angle(hand_angle);
  if (hand_is_lowered())
    hand_lowering = false;
}

bool hand_is_lowered()
{
  return hand_angle == HAND_LOWERED_ANGLE;
}

void raise_hand()
{
  hand_lowering = false;
  hand_angle = HAND_RAISED_ANGLE;
  set_hand_angle(hand_angle);
}
