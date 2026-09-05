#include <Arduino.h>
#include <BallCapture.h>
#include <BallSearch.h>
#include "BallRetrieve.h"

enum class BallRetrieveState : uint8_t
{
  IDLE,
  SEARCHING,
  CLOSING_IN,
  SHIMMYING,
  CAPTURING,
  COMPLETE,
};

BallRetrieveState ball_retrieve_state = BallRetrieveState::IDLE;
uint32_t close_in_started_ms = 0;
uint32_t shimmy_started_ms = 0;

// The ultrasonic sensor rejects very close echoes (under about 3 cm), so use
// a short timed creep to place the ball in the capture mechanism after it
// reaches the 4 cm search threshold. Tune on the assembled robot.
//
// Match BallSearch's reverse speed: on this chassis 25 PWM was found too low
// to reliably overcome static friction while driving backward (see
// SEARCH_REVERSE_SPEED in BallSearch.cpp), so a blind creep at 25 risked
// silently not moving at all before the hand lowers.
constexpr int FINAL_APPROACH_REVERSE_SPEED = 60;
constexpr uint32_t FINAL_APPROACH_REVERSE_MS = 300;

// After the straight creep, the ball tends to sit off-center from the
// grabber. Bias the reverse briefly so the robot curves to one side while
// still backing up, walking the ball into position before the hand lowers.
// Sign/magnitude and duration are both meant to be tuned on the robot: start
// small and increase until the ball lands in the gripper consistently. If it
// shimmies the wrong way, flip the sign of SHIMMY_TURN_BIAS.
constexpr int SHIMMY_TURN_BIAS = 30;
constexpr uint32_t SHIMMY_DURATION_MS = 200;

void begin_ball_retrieval()
{
  // Configure the ultrasonic sensor and raise the hand before the search.
  setup_capture();
  begin_ball_search();
  ball_retrieve_state = BallRetrieveState::SEARCHING;
}

BallRetrieveCommand update_ball_retrieval()
{
  update_hand();

  switch (ball_retrieve_state)
  {
  case BallRetrieveState::IDLE:
    return {0, 0, false, false};

  case BallRetrieveState::SEARCHING:
  {
    const BallSearchCommand search_command = update_ball_search();
    if (search_command.target_found)
    {
      close_in_started_ms = millis();
      ball_retrieve_state = BallRetrieveState::CLOSING_IN;
      return {0, 0, true, false};
    }

    // Retained for callers that may provide a finite search implementation.
    // The current BallSearch implementation rotates until it detects a target.
    if (search_command.complete)
    {
      ball_retrieve_state = BallRetrieveState::COMPLETE;
      return {0, 0, false, false};
    }

    return {search_command.left_speed, search_command.right_speed, false,
            false};
  }

  case BallRetrieveState::CLOSING_IN:
    if (millis() - close_in_started_ms < FINAL_APPROACH_REVERSE_MS)
    {
      return {-FINAL_APPROACH_REVERSE_SPEED, -FINAL_APPROACH_REVERSE_SPEED,
              true, false};
    }

    shimmy_started_ms = millis();
    ball_retrieve_state = BallRetrieveState::SHIMMYING;
    return {-FINAL_APPROACH_REVERSE_SPEED - SHIMMY_TURN_BIAS,
            -FINAL_APPROACH_REVERSE_SPEED + SHIMMY_TURN_BIAS, true, false};

  case BallRetrieveState::SHIMMYING:
    if (millis() - shimmy_started_ms < SHIMMY_DURATION_MS)
    {
      return {-FINAL_APPROACH_REVERSE_SPEED - SHIMMY_TURN_BIAS,
              -FINAL_APPROACH_REVERSE_SPEED + SHIMMY_TURN_BIAS, true, false};
    }

    lower_hand();
    ball_retrieve_state = BallRetrieveState::CAPTURING;
    return {0, 0, true, false};

  case BallRetrieveState::CAPTURING:
    if (hand_is_lowered())
    {
      ball_retrieve_state = BallRetrieveState::COMPLETE;
      return {0, 0, true, true};
    }
    return {0, 0, true, false};

  case BallRetrieveState::COMPLETE:
    return {0, 0, false, false};
  }

  return {0, 0, false, false};
}
