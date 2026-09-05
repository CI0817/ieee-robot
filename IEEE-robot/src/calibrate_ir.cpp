#include <Arduino.h>

// Guided calibration routine for the 3-sensor IR line-following array.
// Flash this (env:calibrate), open the serial monitor at 115200 baud, and
// follow the prompts. Recalibrate whenever lighting, tape, or the sensor
// mount's height/angle changes -- the constants this prints are only valid
// for the exact conditions they were measured under.

constexpr uint8_t LEFT_IR_PIN = 32;
constexpr uint8_t MIDDLE_IR_PIN = 35;
constexpr uint8_t RIGHT_IR_PIN = 34;
constexpr uint8_t SENSOR_COUNT = 3;
constexpr uint8_t SENSOR_PINS[SENSOR_COUNT] = {LEFT_IR_PIN, MIDDLE_IR_PIN, RIGHT_IR_PIN};
const char *const SENSOR_NAMES[SENSOR_COUNT] = {"LEFT", "MIDDLE", "RIGHT"};

constexpr int SAMPLE_COUNT = 300;        // ~1.5s of steady sampling (white step)
constexpr int SWEEP_SAMPLE_COUNT = 800;  // ~4s of continuous sampling while sweeping (black step)
constexpr int SAMPLE_INTERVAL_MS = 5;
constexpr int MIN_SEPARATION = 150;      // flag a sensor if black/white readings overlap this closely

struct SensorStats
{
  int min_val;
  int max_val;
  long sum;
  int sample_count;
};

void wait_for_user(const char *prompt);
void sample_all(SensorStats out[SENSOR_COUNT], int count);
void print_stats(const char *label, SensorStats s);
void report(const char *name, SensorStats white, SensorStats black);

SensorStats white_stats[SENSOR_COUNT];
SensorStats black_stats[SENSOR_COUNT];

void setup()
{
  Serial.begin(9600);
  delay(500);
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  Serial.println();
  Serial.println("=== IR sensor calibration ===");
  Serial.println("Keep the sensor array flat and at riding height throughout.");

  wait_for_user("STEP 1: Hold all three sensors steady over WHITE track material.");
  sample_all(white_stats, SAMPLE_COUNT);
  Serial.println("-- White readings (steady) --");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    print_stats(SENSOR_NAMES[i], white_stats[i]);
  }

  // The array is wider than the tape, so there's no single position where
  // all three sensors sit on black at once -- sweep across it instead and
  // track each channel's peak as it individually passes over the tape.
  wait_for_user("STEP 2: Slowly sweep the array back and forth ACROSS the black tape\n"
                "  (side to side) for the next few seconds, so each sensor individually\n"
                "  crosses directly over the tape at least once.");
  sample_all(black_stats, SWEEP_SAMPLE_COUNT);
  Serial.println("-- Black readings (swept) --");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    print_stats(SENSOR_NAMES[i], black_stats[i]);
  }

  Serial.println();
  Serial.println("=== Suggested constants (paste into main.cpp) ===");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    report(SENSOR_NAMES[i], white_stats[i], black_stats[i]);
  }

  Serial.println();
  Serial.println("Recalibrate whenever lighting, tape, or mounting height/angle changes.");
  Serial.println("Live readout below -- sweep across a tape edge to sanity-check the thresholds.");
}

void loop()
{
  Serial.print("L:");
  Serial.print(analogRead(LEFT_IR_PIN));
  Serial.print(" M:");
  Serial.print(analogRead(MIDDLE_IR_PIN));
  Serial.print(" R:");
  Serial.println(analogRead(RIGHT_IR_PIN));
  delay(100);
}

void wait_for_user(const char *prompt)
{
  Serial.println(prompt);
  Serial.println("  (send any character in the serial monitor to continue)");
  while (!Serial.available())
  {
    delay(10);
  }
  while (Serial.available())
  {
    Serial.read(); // flush whatever was sent
  }
}

void sample_all(SensorStats out[SENSOR_COUNT], int count)
{
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
  {
    out[i] = {4095, 0, 0, count};
  }
  for (int sample = 0; sample < count; sample++)
  {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
      int v = analogRead(SENSOR_PINS[i]);
      out[i].min_val = min(out[i].min_val, v);
      out[i].max_val = max(out[i].max_val, v);
      out[i].sum += v;
    }
    delay(SAMPLE_INTERVAL_MS);
  }
}

void print_stats(const char *label, SensorStats s)
{
  Serial.print(label);
  Serial.print(": min=");
  Serial.print(s.min_val);
  Serial.print(" max=");
  Serial.print(s.max_val);
  Serial.print(" avg=");
  Serial.println(s.sum / s.sample_count);
}

void report(const char *name, SensorStats white, SensorStats black)
{
  // The black reading comes from a sweep, so its min_val is mostly the same
  // off-tape floor as white and isn't meaningful -- compare peaks instead:
  // how much darker did this sensor get at its darkest, versus how bright
  // does it ever falsely read while off the tape.
  int separation = black.max_val - white.max_val;

  // Threshold at the midpoint between the worst-case white reading and the
  // peak black reading, so noise on either surface has equal margin before
  // it crosses the line.
  int threshold = white.max_val + separation / 2;
  int black_max = black.max_val;

  Serial.print("constexpr int ");
  Serial.print(name);
  Serial.print("_BLACK_THRESHOLD = ");
  Serial.print(threshold);
  Serial.println(";");

  Serial.print("constexpr int ");
  Serial.print(name);
  Serial.print("_BLACK_MAX = ");
  Serial.print(black_max);
  Serial.println(";");

  if (separation < MIN_SEPARATION)
  {
    Serial.print("  WARNING: ");
    Serial.print(name);
    Serial.print(" separation is only ");
    Serial.print(separation);
    Serial.println(" -- black/white are not well separated, or this sensor never crossed the tape during the sweep. Check height, angle, tilt, lighting, or repeat the sweep.");
  }
}
