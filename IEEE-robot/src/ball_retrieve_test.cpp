#include <Arduino.h>
#include <BallRetrieve.h>
#include <BallSearch.h>

// Standalone on-robot test for: search -> stop at target -> lower hand.
constexpr int MAX_PWM = 250;
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;

const int left_motorA = 19;
const int left_motorB = 18;
const int right_motorA = 17;
const int right_motorB = 5;
const int left_pwm = 21;
const int right_pwm = 16;

uint32_t last_status_ms = 0;

void set_motor(int pwm_pin, int direction_pin_1, int direction_pin_2,
               int velocity)
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
    digitalWrite(direction_pin_1, LOW);
    digitalWrite(direction_pin_2, LOW);
  }

  const uint8_t pwm_channel =
      pwm_pin == left_pwm ? LEFT_PWM_CHANNEL : RIGHT_PWM_CHANNEL;
  ledcWrite(pwm_channel, pwm);
}

void drive_motors(int left_speed, int right_speed)
{
  set_motor(left_pwm, left_motorA, left_motorB, left_speed);
  set_motor(right_pwm, right_motorA, right_motorB, right_speed);
}

void setup()
{
  Serial.begin(115200);

  pinMode(left_motorA, OUTPUT);
  pinMode(left_motorB, OUTPUT);
  pinMode(right_motorA, OUTPUT);
  pinMode(right_motorB, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(right_pwm, OUTPUT);

  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(left_pwm, LEFT_PWM_CHANNEL);
  ledcAttachPin(right_pwm, RIGHT_PWM_CHANNEL);

  set_ball_search_debug(true);
  begin_ball_retrieval();
  Serial.println("Ball retrieve test started.");
}

void loop()
{
  const BallRetrieveCommand command = update_ball_retrieval();
  drive_motors(command.left_speed, command.right_speed);

  const uint32_t now_ms = millis();
  if (now_ms - last_status_ms >= 250)
  {
    last_status_ms = now_ms;
    Serial.print("Motor command: ");
    Serial.print(command.left_speed);
    Serial.print(", ");
    Serial.print(command.right_speed);
    Serial.print("; ball found: ");
    Serial.print(command.ball_found ? "yes" : "no");
    Serial.print("; capture complete: ");
    Serial.println(command.capture_complete ? "yes" : "no");
  }
}
