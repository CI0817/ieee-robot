#include <Arduino.h>

// Standalone motor-direction test -- bypasses any software polarity
// correction so you can see each motor's RAW physical response to a
// logical "forward"/"backward" command. Use this to figure out what
// LEFT_MOTOR_DIRECTION / RIGHT_MOTOR_DIRECTION (in main.cpp) should each
// be (+1 or -1), independent of whatever's currently set there.
//
// Flash this (env:drive_test), open the serial monitor at 115200 baud.
// PUT THE ROBOT ON A STAND FIRST so the wheels can spin freely without
// driving it off the bench. Send a single character to run a test; the
// motor(s) run continuously until you send another command.
//
//   1 = LEFT motor,  logical FORWARD
//   2 = LEFT motor,  logical BACKWARD
//   3 = RIGHT motor, logical FORWARD
//   4 = RIGHT motor, logical BACKWARD
//   5 = BOTH motors, logical FORWARD  (straight-line check)
//   s = STOP (always safe to send)

constexpr int LEFT_MOTOR_A_PIN = 19;
constexpr int LEFT_MOTOR_B_PIN = 18;
constexpr int RIGHT_MOTOR_A_PIN = 17;
constexpr int RIGHT_MOTOR_B_PIN = 5;
constexpr int LEFT_PWM_PIN = 21;
constexpr int RIGHT_PWM_PIN = 16;

constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr uint8_t LEFT_PWM_CHANNEL = 0;
constexpr uint8_t RIGHT_PWM_CHANNEL = 1;

constexpr int TEST_PWM = 120; // slow enough to watch clearly, not full speed

void raw_set_motor(int direction_pin_1, int direction_pin_2, uint8_t pwm_channel, int velocity);
void stop_all();
void run_command(char command);

void setup()
{
  Serial.begin(115200);
  delay(300);

  pinMode(LEFT_MOTOR_A_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_B_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_A_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_B_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_PWM_CHANNEL);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_PWM_CHANNEL);

  stop_all();

  Serial.println();
  Serial.println("=== Motor direction test (raw -- no polarity correction applied) ===");
  Serial.println("Put the robot on a stand so the wheels can spin freely.");
  Serial.println("Send one character. The motor(s) run continuously until the next command.");
  Serial.println("  1 = LEFT forward    2 = LEFT backward");
  Serial.println("  3 = RIGHT forward   4 = RIGHT backward");
  Serial.println("  5 = BOTH forward (does the robot roll straight?)");
  Serial.println("  s = STOP");
}

void loop()
{
  if (Serial.available())
  {
    char command = Serial.read();
    run_command(command);
  }
  delay(10);
}

void run_command(char command)
{
  switch (command)
  {
  case '1':
    Serial.println("LEFT motor: logical FORWARD -- which way does the wheel actually spin?");
    raw_set_motor(LEFT_MOTOR_A_PIN, LEFT_MOTOR_B_PIN, LEFT_PWM_CHANNEL, TEST_PWM);
    break;

  case '2':
    Serial.println("LEFT motor: logical BACKWARD -- which way does the wheel actually spin?");
    raw_set_motor(LEFT_MOTOR_A_PIN, LEFT_MOTOR_B_PIN, LEFT_PWM_CHANNEL, -TEST_PWM);
    break;

  case '3':
    Serial.println("RIGHT motor: logical FORWARD -- which way does the wheel actually spin?");
    raw_set_motor(RIGHT_MOTOR_A_PIN, RIGHT_MOTOR_B_PIN, RIGHT_PWM_CHANNEL, TEST_PWM);
    break;

  case '4':
    Serial.println("RIGHT motor: logical BACKWARD -- which way does the wheel actually spin?");
    raw_set_motor(RIGHT_MOTOR_A_PIN, RIGHT_MOTOR_B_PIN, RIGHT_PWM_CHANNEL, -TEST_PWM);
    break;

  case '5':
    Serial.println("BOTH motors: logical FORWARD -- does the robot roll straight forward?");
    raw_set_motor(LEFT_MOTOR_A_PIN, LEFT_MOTOR_B_PIN, LEFT_PWM_CHANNEL, TEST_PWM);
    raw_set_motor(RIGHT_MOTOR_A_PIN, RIGHT_MOTOR_B_PIN, RIGHT_PWM_CHANNEL, TEST_PWM);
    break;

  case 's':
  case 'S':
    Serial.println("STOP");
    stop_all();
    break;

  default:
    // Ignore newlines and anything else.
    break;
  }
}

void raw_set_motor(int direction_pin_1, int direction_pin_2, uint8_t pwm_channel, int velocity)
{
  velocity = constrain(velocity, -255, 255);
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

  ledcWrite(pwm_channel, pwm);
}

void stop_all()
{
  raw_set_motor(LEFT_MOTOR_A_PIN, LEFT_MOTOR_B_PIN, LEFT_PWM_CHANNEL, 0);
  raw_set_motor(RIGHT_MOTOR_A_PIN, RIGHT_MOTOR_B_PIN, RIGHT_PWM_CHANNEL, 0);
}
