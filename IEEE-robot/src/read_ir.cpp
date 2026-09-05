#include <Arduino.h>

constexpr uint8_t IR_SENSOR_1_PIN = 0;
constexpr uint8_t IR_SENSOR_2_PIN = 1;
constexpr uint8_t IR_SENSOR_3_PIN = 2;

void setup() {
	Serial.begin(115200);
}

void loop() {
	Serial.print("IR1: ");
	Serial.print(analogRead(IR_SENSOR_1_PIN));
	Serial.print(" | IR2: ");
	Serial.print(analogRead(IR_SENSOR_2_PIN));
	Serial.print(" | IR3: ");
	Serial.println(analogRead(IR_SENSOR_3_PIN));

	delay(100);
}
