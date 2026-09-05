#include <Arduino.h>
#include <BallCapture.h>
#include "BallSearch.h"

// Tune these on the assembled robot.
// Rotate gently while looking for the ball.
constexpr int SEARCH_TURN_SPEED = 30;
// After locating an object, reverse toward the ball/capture side of the robot.
constexpr int SEARCH_REVERSE_SPEED = 45;
constexpr uint32_t SEARCH_TARGET_LOST_MS = 400;
constexpr uint32_t SEARCH_SONAR_INTERVAL_MS = 120;
constexpr float SEARCH_DETECTION_DISTANCE_CM = 15.0f;
// Continue turning briefly after the first echo. This shifts a target seen at
// the edge of the ultrasonic cone closer to the centre of the sensor view.
constexpr uint32_t TARGET_CENTER_TURN_MS = 120;

enum class BallSearchState : uint8_t
{
  IDLE,
  SWEEPING,
  CENTERING_TARGET,
  APPROACHING,
  TARGET_FOUND,
  COMPLETE,
};

BallSearchState ball_search_state = BallSearchState::IDLE;
uint32_t ball_search_state_started_ms = 0;
uint32_t last_sonar_read_ms = 0;
bool ball_search_debug = false;

void debug_log(const char *message)
{
  if (ball_search_debug)
    Serial.println(message);
}

bool poll_for_ball(float &distance_cm)
{
  const uint32_t now_ms = millis();
  if (now_ms - last_sonar_read_ms < SEARCH_SONAR_INTERVAL_MS)
    return false;

  last_sonar_read_ms = now_ms;
  distance_cm = read_distance_cm();
  if (ball_search_debug)
  {
    Serial.print("[BallSearch] distance_cm=");
    Serial.print(distance_cm);
    Serial.print(" candidate=");
    Serial.println(distance_cm > 0.0f &&
                           distance_cm <= SEARCH_DETECTION_DISTANCE_CM
                       ? "yes"
                       : "no");
  }

  // The ultrasonic sensor detects an object, not specifically a ping-pong
  // ball. Keep this range short enough to reduce detections of zone walls.
  return distance_cm > 0.0f && distance_cm <= SEARCH_DETECTION_DISTANCE_CM;
}

void set_ball_search_debug(bool enabled)
{
  ball_search_debug = enabled;
}

void begin_ball_search()
{
  ball_search_state = BallSearchState::SWEEPING;
  ball_search_state_started_ms = millis();
  last_sonar_read_ms = 0;
  debug_log("[BallSearch] start: rotating until an object is detected");
}

BallSearchCommand update_ball_search()
{
  const uint32_t now_ms = millis();
  float distance_cm = -1.0f;

  switch (ball_search_state)
  {
  case BallSearchState::IDLE:
    return {0, 0, false, false};

  case BallSearchState::SWEEPING:
    if (poll_for_ball(distance_cm))
    {
      if (distance_cm <= BALL_DISTANCE_CM)
      {
        ball_search_state = BallSearchState::TARGET_FOUND;
        debug_log("[BallSearch] close target: stop and begin capture");
        return {0, 0, true, true};
      }

      ball_search_state = BallSearchState::CENTERING_TARGET;
      ball_search_state_started_ms = now_ms;
      debug_log("[BallSearch] target detected: centering before reverse");
      return {-SEARCH_TURN_SPEED, SEARCH_TURN_SPEED, false, false};
    }

    // Keep rotating indefinitely until the ultrasonic sensor sees an object.
    return {-SEARCH_TURN_SPEED, SEARCH_TURN_SPEED, false, false};

  case BallSearchState::CENTERING_TARGET:
    if (now_ms - ball_search_state_started_ms >= TARGET_CENTER_TURN_MS)
    {
      ball_search_state = BallSearchState::APPROACHING;
      ball_search_state_started_ms = now_ms;
      debug_log("[BallSearch] centered target: reversing toward it");
      return {0, 0, false, false};
    }
    return {-SEARCH_TURN_SPEED, SEARCH_TURN_SPEED, false, false};

  case BallSearchState::APPROACHING:
    if (poll_for_ball(distance_cm))
    {
      ball_search_state_started_ms = now_ms;
      if (distance_cm <= BALL_DISTANCE_CM)
      {
        ball_search_state = BallSearchState::TARGET_FOUND;
        debug_log("[BallSearch] close target reached: stop and begin capture");
        return {0, 0, false, false};
      }
      return {-SEARCH_REVERSE_SPEED, -SEARCH_REVERSE_SPEED, false, false};
    }

    if (now_ms - ball_search_state_started_ms >= SEARCH_TARGET_LOST_MS)
    {
      ball_search_state = BallSearchState::SWEEPING;
      ball_search_state_started_ms = now_ms;
      debug_log("[BallSearch] target lost: resume continuous rotation");
      return {0, 0, false, false};
    }
    return {-SEARCH_REVERSE_SPEED, -SEARCH_REVERSE_SPEED, false, false};

  case BallSearchState::TARGET_FOUND:
    return {0, 0, true, true};

  case BallSearchState::COMPLETE:
    return {0, 0, false, true};
  }

  return {0, 0, false, true};
}
