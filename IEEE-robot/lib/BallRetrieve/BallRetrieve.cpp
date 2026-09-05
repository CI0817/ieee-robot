#include <Arduino.h>
#include <BallCapture.h>
#include <BallSearch.h>
#include "BallRetrieve.h"

enum class BallRetrieveState : uint8_t
{
  IDLE,
  SEARCHING,
  CLOSING_IN,
  CAPTURING,
  COMPLETE,
};

BallRetrieveState ball_retrieve_state = BallRetrieveState::IDLE;
uint32_t close_in_started_ms = 0;

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
