#pragma once

constexpr int ULTRASONIC_ECHO_PIN = 12;
constexpr int ULTRASONIC_TRIGGER_PIN = 13;
constexpr int CAPTURE_SERVO_PIN = 14;

// Starting values only: adjust for the sensor position and hand linkage.
constexpr float BALL_DISTANCE_CM = 4.0f;
constexpr int HAND_RAISED_ANGLE = 90;              // Initial / release position.
constexpr int HAND_LOWERED_ANGLE = 0;              // Capture position; tune for the linkage.
constexpr unsigned long HAND_STEP_INTERVAL_MS = 5; // lower means faster drop

void setup_capture();
// Returns -1 if no valid echo arrives. Blocks for up to about 120 ms.
float read_distance_cm();
// Detects any object in range, not specifically a ping-pong ball.
bool ball_detected(float max_distance_cm = BALL_DISTANCE_CM);
// Starts gradual lowering. Call update_hand() frequently in loop() to move it.
void lower_hand();
void update_hand();
// Reports commanded position, not physical feedback or capture success.
bool hand_is_lowered();
// Cancels lowering and commands the ready position immediately.
void raise_hand();
