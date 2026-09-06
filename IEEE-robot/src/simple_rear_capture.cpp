#include <Arduino.h>
#include <NewPing.h>

// RCWL-1601 pins. The sensor is mounted at the rear of the robot.
constexpr uint8_t SONAR_TRIGGER_PIN = 13;
constexpr uint8_t SONAR_ECHO_PIN = 12;
constexpr uint16_t SONAR_MAX_DISTANCE_CM = 450;

constexpr uint8_t CAPTURE_SERVO_PIN = 14;
constexpr uint8_t SERVO_CHANNEL = 2;
constexpr uint8_t SERVO_RESOLUTION = 16;
constexpr uint32_t SERVO_PERIOD_US = 20000;
constexpr int SERVO_READY_ANGLE = 0;
constexpr int SERVO_CAPTURE_ANGLE = 90;

// Tune these three values on the assembled robot.
constexpr int DETECT_DISTANCE_CM = 15;
constexpr int TURN_SPEED = 35;
constexpr int REVERSE_SPEED = 35;
constexpr uint32_t REVERSE_TIME_MS = 500;

constexpr int MAX_PWM = 250;
constexpr uint32_t MOTOR_PWM_FREQUENCY = 20000;
constexpr uint8_t MOTOR_PWM_RESOLUTION = 8;
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;

constexpr uint8_t LEFT_MOTOR_A_PIN = 19;
constexpr uint8_t LEFT_MOTOR_B_PIN = 18;
constexpr uint8_t RIGHT_MOTOR_A_PIN = 17;
constexpr uint8_t RIGHT_MOTOR_B_PIN = 5;
constexpr uint8_t LEFT_PWM_PIN = 21;
constexpr uint8_t RIGHT_PWM_PIN = 16;

NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, SONAR_MAX_DISTANCE_CM);

enum class State : uint8_t
{
  SEARCHING,
  REVERSING,
  CAPTURED
};
State state = State::SEARCHING;
uint32_t reverse_started_ms = 0;

void setMotor(uint8_t pwm_channel, uint8_t direction_a, uint8_t direction_b,
              int speed)
{
  speed = constrain(speed, -MAX_PWM, MAX_PWM);
  digitalWrite(direction_a, speed > 0 ? HIGH : LOW);
  digitalWrite(direction_b, speed < 0 ? HIGH : LOW);
  ledcWrite(pwm_channel, abs(speed));
}

void drive(int left_speed, int right_speed)
{
  setMotor(LEFT_PWM_CHANNEL, LEFT_MOTOR_A_PIN, LEFT_MOTOR_B_PIN, left_speed);
  setMotor(RIGHT_PWM_CHANNEL, RIGHT_MOTOR_A_PIN, RIGHT_MOTOR_B_PIN, right_speed);
}

void setServoAngle(int angle)
{
  const int pulse_us = map(constrain(angle, 0, 180), 0, 180, 1000, 2000);
  const uint32_t duty = static_cast<uint32_t>(pulse_us) * 65535 / SERVO_PERIOD_US;
  ledcWrite(SERVO_CHANNEL, duty);
}

int distanceCm()
{
  const unsigned int echo_us = sonar.ping();
  if (echo_us == 0)
    return -1; // No echo.
  return echo_us / US_ROUNDTRIP_CM;
}

void setup()
{
  Serial.begin(115200);

  pinMode(LEFT_MOTOR_A_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_B_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_A_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_B_PIN, OUTPUT);
  ledcSetup(LEFT_PWM_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CHANNEL, MOTOR_PWM_FREQUENCY, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_PWM_CHANNEL);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CHANNEL);

  ledcSetup(SERVO_CHANNEL, 50, SERVO_RESOLUTION);
  ledcAttachPin(CAPTURE_SERVO_PIN, SERVO_CHANNEL);
  setServoAngle(SERVO_READY_ANGLE);

  drive(0, 0);
  delay(500); // Let the servo reach its ready position.
  Serial.println("Simple rear capture ready.");
}

void loop()
{
  if (state == State::SEARCHING)
  {
    // Pivot until the rear sensor sees an object in capture range.
    drive(-TURN_SPEED, TURN_SPEED);

    const int distance_cm = distanceCm();
    if (distance_cm > 0 && distance_cm <= DETECT_DISTANCE_CM)
    {
      drive(0, 0);
      reverse_started_ms = millis();
      state = State::REVERSING;
      Serial.println("Object detected; reversing toward capture mechanism.");
    }

    delay(60); // Keep ultrasonic pings well spaced apart.
    return;
  }

  if (state == State::REVERSING)
  {
    // Negative speeds are backward for this robot's motor wiring.
    drive(-REVERSE_SPEED, -REVERSE_SPEED);
    if (millis() - reverse_started_ms >= REVERSE_TIME_MS)
    {
      drive(0, 0);
      setServoAngle(SERVO_CAPTURE_ANGLE);
      state = State::CAPTURED;
      Serial.println("Capture servo commanded to 0 degrees.");
    }
    return;
  }

  drive(0, 0); // Stay stopped after one capture attempt.
}
