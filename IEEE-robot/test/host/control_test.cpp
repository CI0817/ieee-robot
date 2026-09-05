// Run from IEEE-robot:
// c++ -std=c++11 -Wall -Wextra -Werror -I test/host test/host/control_test.cpp -o /tmp/ieee-control-test
// /tmp/ieee-control-test
#include <cassert>
#include <iostream>
#include "../../src/main.cpp"
unsigned long fake_ms = 0;
int fake_ir[40] = {};
int read_count = 0;

void tick(int left, int middle, int right)
{
  fake_ir[left_ir] = left;
  fake_ir[middle_ir] = middle;
  fake_ir[right_ir] = right;
  fake_ms += CONTROL_PERIOD_MS;
  const int old_reads = read_count;
  const int old_left = last_left_speed, old_right = last_right_speed;
  loop();
  assert(read_count == old_reads + 3);
  assert(abs(last_left_speed) <= MAX_PWM && abs(last_right_speed) <= MAX_PWM);
  if (drive_mode != DriveMode::STOPPED) {
    assert(abs(last_left_speed - old_left) <= PWM_STEP);
    assert(abs(last_right_speed - old_right) <= PWM_STEP);
    assert(old_left * last_left_speed >= 0);
    assert(old_right * last_right_speed >= 0);
  }
}

int main()
{
  setup();
  for (int i = 0; i < 300; ++i) tick(0, 0, 0);
  assert(drive_mode == DriveMode::STOPPED);
  assert(last_left_speed == 0 && last_right_speed == 0);
  tick(0, 105, 0);
  tick(0, 105, 0);
  assert(drive_mode == DriveMode::STOPPED);
  for (int i = 0; i < 20; ++i) tick(0, 105, 0);
  assert(drive_mode == DriveMode::TRACK);
  assert(last_left_speed == STRAIGHT_SPEED && last_right_speed == STRAIGHT_SPEED);
  int reads = read_count;
  loop(); // A busy loop cannot accelerate the controller or its debounce.
  assert(read_count == reads);

  for (int i = 0; i < 30; ++i) tick(620, 0, 0);
  assert(last_left_speed < last_right_speed && last_left_speed >= 0);
  assert(last_line_side == -1);
  for (int i = 0; i < 30; ++i) tick(0, 0, 520);
  assert(last_left_speed > last_right_speed && last_right_speed >= 0);
  assert(last_line_side == 1);

  tick(0, 0, 0);
  assert(drive_mode == DriveMode::GAP);
  for (int i = 0; i < 5; ++i) tick(0, 0, 0);
  tick(0, 105, 0);
  tick(0, 105, 0);
  assert(drive_mode == DriveMode::GAP);
  tick(0, 105, 0);
  assert(drive_mode == DriveMode::TRACK);

  for (int i = 0; i < 30; ++i) tick(0, 0, 520);
  tick(0, 0, 0);
  const unsigned long lost_at = fake_ms;
  while (fake_ms - lost_at < 550) tick(0, 0, 0);
  assert(drive_mode == DriveMode::SEARCH); // No old 500 ms stop.
  assert(last_left_speed > 0 && last_right_speed < 0);
  while (fake_ms - lost_at < 1050) tick(0, 0, 0);
  assert(last_left_speed < 0 && last_right_speed > 0);
  while (fake_ms - lost_at < SEARCH_TIMEOUT_MS) {
    // Isolated sightings must not perpetually extend the search.
    tick(0, (fake_ms / CONTROL_PERIOD_MS) % 10 == 0 ? 105 : 0, 0);
  }
  assert(drive_mode == DriveMode::STOPPED);
  assert(last_left_speed == 0 && last_right_speed == 0);
  for (int i = 0; i < 3; ++i) tick(0, 105, 0);
  assert(drive_mode == DriveMode::TRACK);
  assert(last_left_speed <= PWM_STEP && last_right_speed <= PWM_STEP);
  // Lower exit threshold holds visibility across small boundary noise.
  tick(0, 58, 0);
  assert(drive_mode == DriveMode::TRACK);
  tick(0, 0, 0);
  assert(drive_mode == DriveMode::GAP);
  std::cout << "Control behavior tests passed\n";
}
