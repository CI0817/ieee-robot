// Host regression tests: see README.md for the compile/run command.
#include <cassert>
#include <iostream>
#include "../../src/main.cpp"
uint32_t test_ms = 0;
int readings[40] = {};

void tick(int left, int middle, int right)
{
  readings[left_ir] = left;
  readings[middle_ir] = middle;
  readings[right_ir] = right;
  test_ms += CONTROL_PERIOD_MS;
  loop();
}
void reset_controller()
{
  drive_state = DriveState::Follow;
  last_direction = turn_direction = corner_direction = corner_samples = centered_samples = 0;
  reset_steering();
  visible_samples = 0;
  lost_samples = 0;
  left_sensor.black = middle_sensor.black = right_sensor.black = false;
  left_sensor.strength = middle_sensor.strength = right_sensor.strength = 0;
  drive_motors(0, 0);
}
int main()
{
  setup();
  for (int i = 0; i < 10; ++i) tick(20, 165, 20);
  assert(last_left_speed == straight_speed && last_right_speed == straight_speed);
  // Arcs with center visible must not abruptly switch into reverse.
  for (int i = 0; i < 20; ++i)
  {
    tick(300, 165, 20);
    assert(drive_state == DriveState::Follow);
    assert(last_left_speed >= 0 && last_right_speed >= 0);
    assert(abs(steering) <= MAX_STEERING_PWM);
  }
  assert(last_left_speed < last_right_speed);
  // A centered observation must cancel the previous arc command immediately.
  tick(20, 165, 20);
  assert(last_left_speed == straight_speed && last_right_speed == straight_speed);
  // Crossing the line must not leave a delayed correction toward the old side.
  for (int i = 0; i < 10; ++i) tick(760, 165, 20);
  tick(20, 165, 270);
  assert(last_left_speed > last_right_speed);
  tick(20, 165, 20);
  assert(last_left_speed == last_right_speed);

  // At equal position error, moving toward center should reduce correction.
  reset_steering();
  calculate_steering(0.5f, false, 0.01f);
  const float approaching = calculate_steering(0.4f, false, 0.01f);
  reset_steering();
  const float stationary = calculate_steering(0.4f, false, 0.01f);
  assert(approaching < stationary);
  reset_steering();
  calculate_steering(0.3f, false, 0.01f);
  assert(calculate_steering(0.4f, false, 0.01f) > stationary);
  // Resuming after a long delay must not apply a stale derivative kick.
  assert(abs(calculate_steering(0.4f, false, 0.2f) - stationary) < 0.001f);
  for (int direction : {-1, 1})
  {
    reset_controller();
    tick(20, 165, 20);
    // Moderate black (well below old 90% pivot threshold) must trigger turns.
    for (int i = 0; i < CONFIRM_SAMPLES; ++i)
      tick(direction < 0 ? 200 : 20, 20, direction > 0 ? 120 : 20);
    assert(drive_state == DriveState::Turn);
    assert(last_left_speed == direction * turning_speed);
    tick(20, 20, 20);
    assert(drive_state == DriveState::Turn);
    assert(last_right_speed == -direction * turning_speed);
    tick(20, 165, 20); // A brief center detection must not end the pivot.
    assert(drive_state == DriveState::Turn);
    for (int i = 0; i < 10; ++i) tick(20, 20, 20);
    for (int i = 0; i < CONFIRM_SAMPLES; ++i) tick(20, 165, 20);
    assert(drive_state == DriveState::Follow);
    assert(last_left_speed == last_right_speed);
  }
  reset_controller();
  tick(200, 20, 20);
  tick(200, 20, 20);
  for (int i = 0; i < LOST_CONFIRM_SAMPLES; ++i) tick(20, 20, 20); // Confirm loss.
  assert(drive_state == DriveState::Turn && turn_direction == -1);
  for (int i = 0; i < 130; ++i) tick(20, 20, 20);
  assert(drive_state == DriveState::Stopped);
  assert(last_left_speed == 0 && last_right_speed == 0);
  tick(20, 165, 20);
  assert(drive_state == DriveState::Stopped);
  tick(20, 20, 20); // Noise must reset the restart confirmation.
  for (int i = 0; i < CONFIRM_SAMPLES; ++i) tick(20, 165, 20);
  assert(drive_state == DriveState::Follow);
  assert(last_left_speed > 0 && last_right_speed > 0);
  reset_controller();
  tick(20, 165, 20);
  for (int i = 0; i < LOST_CONFIRM_SAMPLES; ++i) tick(20, 20, 20);
  assert(drive_state == DriveState::Gap);
  assert(last_left_speed == GAP_SPEED && last_right_speed == GAP_SPEED);
  for (int i = 0; i < 5; ++i) tick(20, 20, 20);
  tick(20, 165, 20);
  assert(drive_state == DriveState::Follow);
  // An extended gap enters a bounded search that sweeps both directions.
  for (int i = 0; i < LOST_CONFIRM_SAMPLES; ++i) tick(20, 20, 20);
  for (uint32_t i = 0; i < GAP_CROSS_MS / CONTROL_PERIOD_MS; ++i) tick(20, 20, 20);
  assert(drive_state == DriveState::Search && last_left_speed > 0);
  for (uint32_t i = 0; i < SEARCH_LEG_MS / CONTROL_PERIOD_MS; ++i) tick(20, 20, 20);
  assert(drive_state == DriveState::Search && last_left_speed < 0);
  for (uint32_t i = 0; i < 2 * SEARCH_LEG_MS / CONTROL_PERIOD_MS; ++i) tick(20, 20, 20);
  assert(drive_state == DriveState::Search && last_left_speed > 0);
  for (int i = 0; i < CONFIRM_SAMPLES; ++i) tick(20, 165, 20);
  assert(drive_state == DriveState::Follow);
  for (int i = 0; i < LOST_CONFIRM_SAMPLES; ++i) tick(20, 20, 20);
  for (uint32_t i = 0; i < (GAP_CROSS_MS + SEARCH_TIMEOUT_MS) / CONTROL_PERIOD_MS; ++i)
    tick(20, 20, 20);
  assert(drive_state == DriveState::Stopped);
  assert(last_left_speed == 0 && last_right_speed == 0);
  for (int i = 0; i < CONFIRM_SAMPLES + 2; ++i) tick(200, 20, 20);
  assert(drive_state == DriveState::Turn && turn_direction == -1);
  // Weak side readings must not amplify into large steering or false corners.
  reset_controller();
  for (int i = 0; i < 100; ++i)
  {
    tick(i % 2 ? 46 : 20, 41, i % 2 ? 20 : 46);
    assert(drive_state == DriveState::Follow);
    assert(abs(last_left_speed - last_right_speed) <= 2);
  }
  // Even with the middle briefly absent, near-white side noise is not a corner.
  for (int i = 0; i < 10; ++i) tick(46, 20, 20);
  assert(drive_state == DriveState::Follow);
  assert(last_direction == 0);
  tick(20, 165, 20);
  int before_left = last_left_speed, before_right = last_right_speed;
  for (int i = 0; i < LOST_CONFIRM_SAMPLES - 1; ++i) tick(20, 20, 20);
  assert(drive_state == DriveState::Follow);
  assert(last_left_speed == before_left && last_right_speed == before_right);
  tick(20, 165, 20);
  assert(lost_samples == 0);
  // Center plus a side must release a turn rather than pivot until timeout.
  begin_turn(-1, test_ms);
  for (int i = 0; i < 10; ++i) tick(200, 165, 20);
  assert(drive_state == DriveState::Follow);
  // Threshold chatter stays black until the hysteresis exit is reached.
  reset_controller();
  tick(46, 165, 20);
  tick(44, 165, 20);
  assert(left_sensor.black);
  tick(39, 165, 20);
  assert(!left_sensor.black);
  std::cout << "Turn-control regression tests passed\n";
}
