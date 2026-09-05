#include <Arduino.h>
#include <BallCapture.h>

bool capture_started = false;
bool capture_finished = false;

void setup()
{
  Serial.begin(115200);
  // Keep the drive motors off during this standalone bench test.
  pinMode(21, OUTPUT);
  pinMode(16, OUTPUT);
  digitalWrite(21, LOW);
  digitalWrite(16, LOW);
  setup_capture();
}

void loop()
{
  update_hand();
  // Once capture starts, keep updating the hand without blocking sensor reads.
  // Losing the echo as the ball gets closer must not cancel the capture.
  if (capture_started)
  {
    if (!capture_finished && hand_is_lowered())
    {
      capture_finished = true;
      Serial.println("Capture position commanded. Reset the board to try again.");
    }
    return;
  }

  const float distance_cm = read_distance_cm();
  Serial.print("Distance (cm, -1 = no valid echo): ");
  Serial.println(distance_cm);

  if (distance_cm > 0.0f && distance_cm <= BALL_DISTANCE_CM)
  {
    lower_hand();
    capture_started = true;
    Serial.println("Hand lowering slowly.");
  }
}
