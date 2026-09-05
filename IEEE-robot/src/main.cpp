#include <Arduino.h>

const int black_threshold = 500;
const int white_threshold = 700;

const int turning_speed = 100;
const int straight_speed = 100;

const int left_ir = 1;
const int middle_ir = 2;
const int right_ir = 3;
const int left_motor = 4;
const int right_motor = 5;
bool capacitor_zone = false;

int determine_drive_mode();
void pid_drive();
int calculate_pid_speed(int sensor_pin);
void drive_motors(int left_vel, int right_vel);
bool check_black(int sensor_pin);
void end_zone();
void stop();


void setup() {
  pinMode(left_ir, INPUT);
  pinMode(middle_ir, INPUT);
  pinMode(right_ir, INPUT);
  pinMode(left_motor, OUTPUT);
  pinMode(right_motor, OUTPUT);
}

void loop() {
  determine_drive_mode();
}


int determine_drive_mode() {
  if (check_black(middle_ir)) {

    if (check_black(left_ir) && check_black(right_ir)) { 
      // If all sensors detect black, check if we are in the end zone
      drive_motors(straight_speed, straight_speed); 
      delay(50); // Drive straight a little
      if (check_black(left_ir) && check_black(middle_ir) && check_black(right_ir)) {
        end_zone();
        return 10; // End zone
      }
      else {
        // If not in the end zone, we are in the capacitor zone, drive straight
        capacitor_zone = true;
      }
    } 
    else {
      // If the middle IR sensor detects black, drive in PID mode
      pid_drive();
      return 1; // PID drive mode
    }
  }

  else if (check_black(left_ir) && check_black(right_ir)) {
    // If both left and right IR sensors detect black, drive straight
    drive_motors(straight_speed, straight_speed); // Drive straight
    return 2; // Straight drive mode
  }

  else if (check_black(left_ir)) {
    // If only the left IR sensor detects black, turn left
    drive_motors(-turning_speed, turning_speed); // Turn left
    return 3; // Left turn mode
  } 

  else if (check_black(right_ir)) {
    // If only the right IR sensor detects black, turn right
    drive_motors(turning_speed, -turning_speed); // Turn right
    return 4; // Right turn mode
  }

  else { // No sensors detect black
    if (capacitor_zone) {
      // If we are in the capacitor zone, drive straight
      drive_motors(straight_speed, straight_speed); // Drive straight
      return 5; // Straight drive mode in capacitor zone
    }
    else {
      // Spin to search for the line
      drive_motors(turning_speed, -turning_speed); // Spin in place
      return 0; // Search mode
    }
  }
}

void pid_drive() {
  capacitor_zone = false; // Reset capacitor zone flag

  int left_vel = calculate_pid_speed(left_ir);
  int right_vel = calculate_pid_speed(right_ir);

  drive_motors(left_vel, right_vel); 
}

int calculate_pid_speed(int sensor_pin) {
  int sensor_value = analogRead(sensor_pin);

  // Simple proportional control based on sensor value
  int error = sensor_value - black_threshold;
  int speed = error * 2; // Scale the speed
  return speed;
}

void drive_motors(int left_vel, int right_vel) {
  analogWrite(left_motor, left_vel);
  analogWrite(right_motor, right_vel);
}

bool check_black(int sensor_pin) {
  int sensor_value = analogRead(sensor_pin);
  return sensor_value < black_threshold; // Returns true if the sensor detects black
}

void end_zone() {
  capacitor_zone = false; // Reset capacitor zone flag
  stop(); // Stop the motors
  // Additional logic for end zone can be added here
}

void stop() {
  drive_motors(0, 0); // Stop the motors
}