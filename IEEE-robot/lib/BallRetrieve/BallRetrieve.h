#pragma once

// Motor request and progress state for the combined search-and-capture flow.
struct BallRetrieveCommand
{
  int left_speed;
  int right_speed;
  bool ball_found;
  bool capture_complete;
};

// Raises the hand, then starts the ball search.
void begin_ball_retrieval();

// Search first. Once BallSearch finds a close target, creep backward briefly,
// then stop and lower the hand. Call repeatedly and apply returned speeds.
BallRetrieveCommand update_ball_retrieval();
