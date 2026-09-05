#include <Arduino.h>
#include <BallCapture.h>
#include "BallSearch.h"

// Tune these on the assembled robot.
// A stationary in-place pivot needs more torque than driving or steering does
// (both wheels scrub against the ground with no forward momentum to help), so
// keep this comfortably above the chassis's stall/static-friction threshold
// even though a slower scan would otherwise be preferable.
constexpr int SEARCH_TURN_SPEED = 70;
// After locating an object, reverse toward the ball/capture side of the robot.
constexpr int SEARCH_REVERSE_SPEED = 60;
constexpr uint32_t SEARCH_TARGET_LOST_MS = 400;
constexpr uint32_t SEARCH_SONAR_INTERVAL_MS = 120;
constexpr float SEARCH_DETECTION_DISTANCE_CM = 15.0f;
// Once already committed to a target, keep tracking it with a looser radius
// than the initial detection. A narrow ultrasonic cone can drift off the ball
// as the robot gets closer and the same aiming error becomes a bigger lateral
// miss; a reading a bit beyond the entry cutoff is more likely that drift
// than a brand new object, so treat it as still-tracking rather than lost.
constexpr float SEARCH_TRACKING_DISTANCE_CM = 25.0f;
// If the last confirmed reading was already this close, a subsequent dropout
// or out-of-range echo is treated as the sensor's blind zone or a specular
// bounce off the ball's curved surface, not as the ball actually leaving.
constexpr float SEARCH_ASSUME_FOUND_DISTANCE_CM = 8.0f;
// Guard against reversing forever at a target that keeps reporting valid but
// non-decreasing readings (e.g. a static obstacle, or something receding at
// the same rate the robot approaches). If the distance hasn't meaningfully
// closed within this time, give up and resume sweeping.
constexpr uint32_t SEARCH_APPROACH_TIMEOUT_MS = 4000;
constexpr float SEARCH_APPROACH_MIN_PROGRESS_CM = 2.0f;
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
// Most recent reading that was actually in range, kept even after later
// dropouts/jumps so a lost-target timeout can tell how close we last knew we
// were. -1 means nothing has been confirmed yet this search.
float last_confirmed_distance_cm = -1.0f;
// Distance at the moment APPROACHING began, and when it began, used to detect
// a target that never actually gets closer.
float approach_reference_distance_cm = -1.0f;
uint32_t approach_started_ms = 0;

void debug_log(const char *message)
{
  if (ball_search_debug)
    Serial.println(message);
}

bool poll_for_ball(float &distance_cm, float max_distance_cm)
{
  const uint32_t now_ms = millis();
  if (now_ms - last_sonar_read_ms < SEARCH_SONAR_INTERVAL_MS)
    return false;

  last_sonar_read_ms = now_ms;
  distance_cm = read_distance_cm();
  const bool in_range =
      distance_cm > 0.0f && distance_cm <= max_distance_cm;
  if (ball_search_debug)
  {
    Serial.print("[BallSearch] distance_cm=");
    Serial.print(distance_cm);
    Serial.print(" candidate=");
    Serial.println(in_range ? "yes" : "no");
  }

  // The ultrasonic sensor detects an object, not specifically a ping-pong
  // ball. Keep this range short enough to reduce detections of zone walls.
  return in_range;
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
  last_confirmed_distance_cm = -1.0f;
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
    if (poll_for_ball(distance_cm, SEARCH_DETECTION_DISTANCE_CM))
    {
      last_confirmed_distance_cm = distance_cm;
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
      approach_started_ms = now_ms;
      approach_reference_distance_cm = last_confirmed_distance_cm;
      debug_log("[BallSearch] centered target: reversing toward it");
      return {0, 0, false, false};
    }
    return {-SEARCH_TURN_SPEED, SEARCH_TURN_SPEED, false, false};

  case BallSearchState::APPROACHING:
    if (poll_for_ball(distance_cm, SEARCH_TRACKING_DISTANCE_CM))
    {
      ball_search_state_started_ms = now_ms;
      last_confirmed_distance_cm = distance_cm;
      if (distance_cm <= BALL_DISTANCE_CM)
      {
        ball_search_state = BallSearchState::TARGET_FOUND;
        debug_log("[BallSearch] close target reached: stop and begin capture");
        return {0, 0, false, false};
      }

      // Valid readings keep arriving, but if the distance hasn't meaningfully
      // closed since we started reversing, this isn't a target we can reach
      // (e.g. an obstacle, or something receding at the same rate) - give up
      // rather than reversing indefinitely.
      const bool making_progress =
          approach_reference_distance_cm < 0.0f ||
          (approach_reference_distance_cm - distance_cm) >=
              SEARCH_APPROACH_MIN_PROGRESS_CM;
      if (!making_progress &&
          now_ms - approach_started_ms >= SEARCH_APPROACH_TIMEOUT_MS)
      {
        ball_search_state = BallSearchState::SWEEPING;
        ball_search_state_started_ms = now_ms;
        debug_log("[BallSearch] no progress closing in: resume rotation");
        return {0, 0, false, false};
      }

      return {-SEARCH_REVERSE_SPEED, -SEARCH_REVERSE_SPEED, false, false};
    }

    if (now_ms - ball_search_state_started_ms >= SEARCH_TARGET_LOST_MS)
    {
      if (last_confirmed_distance_cm > 0.0f &&
          last_confirmed_distance_cm <= SEARCH_ASSUME_FOUND_DISTANCE_CM)
      {
        // We were already right on top of it; the dropout/jump is almost
        // certainly the sensor's blind zone or a bad echo off the ball's
        // curved surface, not the ball actually leaving. Stop instead of
        // abandoning the approach and re-sweeping from scratch.
        ball_search_state = BallSearchState::TARGET_FOUND;
        debug_log("[BallSearch] signal lost very close: assume target reached");
        return {0, 0, true, true};
      }

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
