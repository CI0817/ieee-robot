#include <Arduino.h>

constexpr uint8_t IR_SENSOR_1_PIN = 32;
constexpr uint8_t IR_SENSOR_2_PIN = 35;
constexpr uint8_t IR_SENSOR_3_PIN = 34;

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	int sensor1 = analogRead(IR_SENSOR_1_PIN);
	int sensor2 = analogRead(IR_SENSOR_2_PIN);
	int sensor3 = analogRead(IR_SENSOR_3_PIN);

	Serial.print("IR1: ");
	Serial.print(sensor1);
	Serial.print(" | IR2: ");
	Serial.print(sensor2);
	Serial.print(" | IR3: ");
	Serial.println(sensor3);

	delay(100);
}
