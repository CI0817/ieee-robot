#include <Arduino.h>
#include <Preferences.h>

// Positive wheel velocity must mean forward on BOTH sides. Verify with wheels lifted.
constexpr int IR_PINS[] = {32, 35, 34}; // left, centre, right
constexpr int LEFT_A = 19, LEFT_B = 18, RIGHT_A = 17, RIGHT_B = 5;
constexpr int LEFT_PWM = 21, RIGHT_PWM = 16;
constexpr int MAX_PWM = 80;
constexpr int LEFT_TRIM = 0, RIGHT_TRIM = 0; // measured forward PWM offsets
constexpr bool BLACK_IS_HIGH = true; // check raw readings; change if your module is inverted

// Starting values, to tune on the actual robot. PWM is not a speed measurement.
constexpr float BASE_PWM = 40, SLOW_PWM = 30, PIVOT_PWM = 40;
constexpr float KP = 24, KD = 0.12f; // derivative uses seconds; integral intentionally disabled
constexpr float MAX_CORRECTION = 35, DERIVATIVE_TAU = 0.025f;
constexpr uint32_t CONTROL_US = 5000;
constexpr uint32_t GAP_HOLD_MS = 160, SEARCH_TIMEOUT_MS = 1400;
constexpr uint32_t CORNER_CONFIRM_MS = 45, CORNER_TIMEOUT_MS = 1200;
constexpr uint32_t REACQUIRE_MS = 30, CALIBRATION_MS = 6000;
constexpr int MIN_CALIBRATION_SPAN = 300;
constexpr float BLACK_ON = 0.55f, BLACK_OFF = 0.35f;
constexpr float MIN_SIGNAL = 0.30f;

// At ambiguous junctions choose a branch explicitly. This is not a maze solver.
enum class Branch { Left, Right };
constexpr Branch BRANCH_PREFERENCE = Branch::Left;
enum class Mode { Stopped, Calibrating, Tracking, Gap, Searching, Corner };
Mode mode = Mode::Stopped;
Preferences preferences;
int white[3], black[3], calMin[3], calMax[3];
bool calibrated = false, logging = false;
struct Sample { int raw[3]; float b[3]; bool dark[3]; bool visible; float error; };
Sample sample = {};
uint32_t previousTick = 0, stateSince = 0, lostSince = 0, lastLog = 0;
uint32_t candidateSince = 0, centredSince = 0;
bool candidateActive = false, centredActive = false;
int candidateSide = 0, turnSide = 0, lastLineSide = 0;
float previousError = 0, derivative = 0, recentSteering = 0, gapSteering = 0;
float trackingBase = BASE_PWM, gapBase = BASE_PWM;
bool derivativeReady = false;
int commandedLeft = 0, commandedRight = 0;

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
void setMotor(int pwmPin, int a, int b, int velocity)
{
  velocity = constrain(velocity, -MAX_PWM, MAX_PWM);
  digitalWrite(a, velocity > 0 ? HIGH : LOW);
  digitalWrite(b, velocity < 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(velocity));
}
void drive(int left, int right)
{
  // Trim only moving forward wheels; never turn a stop into motion.
  commandedLeft = constrain(left > 0 ? left + LEFT_TRIM : left, -MAX_PWM, MAX_PWM);
  commandedRight = constrain(right > 0 ? right + RIGHT_TRIM : right, -MAX_PWM, MAX_PWM);
  setMotor(LEFT_PWM, LEFT_A, LEFT_B, commandedLeft);
  setMotor(RIGHT_PWM, RIGHT_A, RIGHT_B, commandedRight);
}
void resetPD() { derivativeReady = false; derivative = 0; }
void enter(Mode next, uint32_t now)
{
  mode = next; stateSince = now; centredActive = false; candidateActive = false;
  resetPD();
}
void halt(uint32_t now)
{
  enter(Mode::Stopped, now); drive(0, 0);
}
bool validCalibration()
{
  for (int i = 0; i < 3; ++i)
    if (white[i] < 0 || white[i] > 4095 || black[i] < 0 || black[i] > 4095 ||
        abs(black[i] - white[i]) < MIN_CALIBRATION_SPAN) return false;
  return true;
}
void startCalibration(uint32_t now)
{
  halt(now);
  for (int i = 0; i < 3; ++i) { calMin[i] = 4095; calMax[i] = 0; }
  enter(Mode::Calibrating, now);
  Serial.println("Calibration: sweep EVERY sensor over floor and tape for 6 seconds. Motors stopped.");
}
void arm(uint32_t now)
{
  if (!calibrated) { Serial.println("Calibrate first: send c."); return; }
  recentSteering = 0; lastLineSide = 0;
  trackingBase = BASE_PWM;
  for (int i = 0; i < 3; ++i) sample.dark[i] = false;
  enter(Mode::Tracking, now);
  Serial.println("Running. Send s to stop.");
}
void readSensors()
{
  float total = 0, peak = 0;
  for (int i = 0; i < 3; ++i)
  {
    sample.raw[i] = analogRead(IR_PINS[i]);
    if (!calibrated) continue;
    sample.b[i] = clampf((sample.raw[i] - white[i]) / float(black[i] - white[i]), 0, 1);
    sample.dark[i] = sample.b[i] >= (sample.dark[i] ? BLACK_OFF : BLACK_ON);
    total += sample.b[i]; peak = max(peak, sample.b[i]);
  }
  sample.visible = calibrated && peak >= MIN_SIGNAL;
  sample.error = sample.visible ? (sample.b[2] - sample.b[0]) / total : 0;
}
bool centreAcquired(uint32_t now)
{
  // Require a single centred line, not an all-black patch or split.
  bool centred = sample.dark[1] && !sample.dark[0] && !sample.dark[2];
  if (!centred) { centredActive = false; return false; }
  if (!centredActive) { centredActive = true; centredSince = now; }
  return now - centredSince >= REACQUIRE_MS;
}
void pivot(int side) { drive(int(side * PIVOT_PWM), int(-side * PIVOT_PWM)); }
void track(float dt, bool ambiguous)
{
  float error = sample.error;
  // Multi-line patterns cannot be interpreted using their centroid.
  if (ambiguous) error = BRANCH_PREFERENCE == Branch::Left ? -0.7f : 0.7f;
  if (!ambiguous && fabsf(error) > 0.15f) lastLineSide = error > 0 ? 1 : -1;
  float rawD = derivativeReady ? (error - previousError) / dt : 0;
  derivative += dt / (DERIVATIVE_TAU + dt) * (rawD - derivative);
  previousError = error; derivativeReady = true;
  float correction = clampf(KP * error + KD * derivative, -MAX_CORRECTION, MAX_CORRECTION);
  // History is slower than the instantaneous D term used for stabilization.
  recentSteering += dt / (0.080f + dt) * (correction - recentSteering);
  float base = ambiguous ? SLOW_PWM : BASE_PWM - (BASE_PWM - SLOW_PWM) * fabsf(error);
  trackingBase = base;
  drive(int(clampf(base + correction, 0, MAX_PWM)), int(clampf(base - correction, 0, MAX_PWM)));
}
void control(uint32_t now, float dt)
{
  if (mode == Mode::Stopped) { drive(0, 0); return; }
  if (mode == Mode::Calibrating)
  {
    for (int i = 0; i < 3; ++i) {
      calMin[i] = min(calMin[i], sample.raw[i]); calMax[i] = max(calMax[i], sample.raw[i]);
    }
    if (now - stateSince < CALIBRATION_MS) return;
    for (int i = 0; i < 3; ++i) {
      white[i] = BLACK_IS_HIGH ? calMin[i] : calMax[i];
      black[i] = BLACK_IS_HIGH ? calMax[i] : calMin[i];
    }
    calibrated = validCalibration();
    if (calibrated) {
      size_t savedWhite = preferences.putBytes("white", white, sizeof(white));
      size_t savedBlack = preferences.putBytes("black", black, sizeof(black));
      Serial.println(savedWhite == sizeof(white) && savedBlack == sizeof(black)
                         ? "Calibration saved. Place robot on line; send g to start."
                         : "Calibration valid in RAM but saving failed. Send g to start; recalibrate after reboot.");
    } else Serial.println("Calibration failed: each sensor needs both floor and tape. Send c to retry.");
    halt(now); return;
  }
  if (mode == Mode::Corner)
  {
    if (centreAcquired(now)) { enter(Mode::Tracking, now); recentSteering = 0; }
    else if (now - stateSince >= CORNER_TIMEOUT_MS) { halt(now); Serial.println("Corner timeout; stopped."); return; }
    else { pivot(turnSide); return; }
  }
  if (mode == Mode::Gap || mode == Mode::Searching)
  {
    if (sample.visible && (mode == Mode::Gap || centreAcquired(now))) {
      enter(Mode::Tracking, now);
    } else {
      if (now - lostSince >= GAP_HOLD_MS + SEARCH_TIMEOUT_MS) {
        halt(now); Serial.println("Line lost; stopped. Reposition and send g."); return;
      }
      if (mode == Mode::Gap && now - lostSince >= GAP_HOLD_MS) enter(Mode::Searching, now);
      if (mode == Mode::Gap) {
        constexpr float scale = 0.65f;
        drive(int(scale * clampf(gapBase + gapSteering, 0, MAX_PWM)),
              int(scale * clampf(gapBase - gapSteering, 0, MAX_PWM)));
      } else {
        // Search the remembered side first, then sweep back. Even zero history recovers.
        int side = lastLineSide != 0 ? lastLineSide : (BRANCH_PREFERENCE == Branch::Left ? -1 : 1);
        pivot(now - stateSince < 450 ? side : -side);
      }
      return;
    }
  }
  if (!sample.visible)
  {
    lostSince = now; gapSteering = recentSteering; gapBase = trackingBase;
    enter(Mode::Gap, now);
    drive(int(0.65f * clampf(gapBase + gapSteering, 0, MAX_PWM)),
          int(0.65f * clampf(gapBase - gapSteering, 0, MAX_PWM)));
    return;
  }
  bool ambiguous = sample.dark[0] && sample.dark[2];
  int outer = !sample.dark[1] && !ambiguous ? (sample.dark[2] ? 1 : (sample.dark[0] ? -1 : 0)) : 0;
  if (outer != 0) {
    if (!candidateActive || candidateSide != outer) {
      candidateActive = true; candidateSide = outer; candidateSince = now;
    }
    if (now - candidateSince >= CORNER_CONFIRM_MS) {
      turnSide = outer; lastLineSide = outer; recentSteering = outer * MAX_CORRECTION;
      enter(Mode::Corner, now); pivot(turnSide); return;
    }
  } else candidateActive = false;
  track(dt, ambiguous);
}
void setup()
{
  Serial.begin(115200);
  for (int pin : IR_PINS) pinMode(pin, INPUT);
  for (int pin : {LEFT_A, LEFT_B, RIGHT_A, RIGHT_B, LEFT_PWM, RIGHT_PWM}) pinMode(pin, OUTPUT);
  analogReadResolution(12);
  drive(0, 0);
  preferences.begin("line-cal", false);
  preferences.getBytes("white", white, sizeof(white));
  preferences.getBytes("black", black, sizeof(black));
  calibrated = validCalibration();
  Serial.println("Commands: c=calibrate, g=go, s=stop, l=toggle logging. Always starts stopped.");
  Serial.println(calibrated ? "Saved calibration loaded. Send g when ready." : "No valid calibration. Send c.");
  previousTick = micros();
}
void loop()
{
  uint32_t now = millis();
  // Limit serial work so a burst of input cannot starve the controller.
  if (Serial.available()) {
    switch (Serial.read()) {
      case 'c': startCalibration(now); break;
      case 'g': if (mode != Mode::Calibrating) arm(now); break;
      case 's': halt(now); break;
      case 'l': logging = !logging; break;
    }
  }
  uint32_t tick = micros(), elapsed = tick - previousTick;
  if (elapsed < CONTROL_US) return;
  previousTick = tick;
  // A late cycle invalidates derivative history; use the actual elapsed time.
  if (elapsed > 4 * CONTROL_US) resetPD();
  readSensors();
  control(now, elapsed / 1000000.0f);
  if (logging && now - lastLog >= 100 && Serial.availableForWrite() >= 100) {
    lastLog = now;
    Serial.printf("%lu mode=%d raw=%d,%d,%d e=%.3f pwm=%d,%d\n",
                  (unsigned long)now, int(mode), sample.raw[0], sample.raw[1], sample.raw[2],
                  sample.error, commandedLeft, commandedRight);
  }
}
