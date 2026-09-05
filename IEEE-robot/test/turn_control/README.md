# Turn-control regression tests

Run from the repository root using a host C++ compiler:

```sh
c++ -std=c++11 -Wall -Wextra -I IEEE-robot/test/turn_control IEEE-robot/test/turn_control/test.cpp -o /tmp/ieee-turn-test
/tmp/ieee-turn-test
```

If Apple's compiler cannot find its C++ headers, add
`-isystem /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/c++/v1`.

The Arduino stub supplies sensor readings and time to the actual firmware loop.
Tests cover centered travel, smooth arc steering, both corner directions at
moderate black readings, holding a turn through line loss, stable center
reacquisition, recovery timeout, short gap crossing, bidirectional search,
automatic restart on stable line detection, hysteresis, weak-signal steering,
brief dropouts, turn completion with adjacent tape overlap, immediate centering,
steering direction after crossing the line, and derivative damping/reset.
They do not model chassis motion, motor torque, or optical sensor geometry.

On the robot, verify that centered tape reliably activates the middle sensor:
stable middle detection is the turn-completion condition. Confirm that positive
PWM drives each wheel forward. Test left and right arcs and corners at the default
100 cruise PWM. Adjust turning_speed if a wheel stalls while pivoting; tune
MIN_TURN_MS and TURN_TIMEOUT_MS against observed turns. The supplied sensor
thresholds and maxima are retained.

Three consecutive all-white samples confirm line loss; shorter dropouts hold
the previous motor command. Confirmed loss with no recent side direction drives straight at GAP_SPEED for up to
GAP_CROSS_MS (180 ms), then searches right/left/right for up to 1500 ms. These
timings are starting values and depend on chassis speed and tape geometry.
Confirmed loss with a known side pivots toward that side, with a 1200 ms
timeout. After either timeout, the robot stops but keeps reading sensors; three
consecutive visible-line samples resume control without reset. A single noisy
sample does not restart it. Junction route selection is not implemented.

Normal steering uses PD on unfiltered right-minus-left strength, with a continuous
0.03 deadband. KP is 55 PWM/error and KD is 0.8 PWM-seconds/error. Only the
derivative is filtered (20 ms time constant), and its contribution is capped at
20 PWM. The total correction is capped at 55 PWM. Center-only detection immediately
clears the correction and derivative history. The previous whole-signal filter
and output slew limiter are removed, so they cannot preserve an old turn command
after centering. These gains are starting values, not hardware-validated tuning.

There is no integral term. Control uses measured elapsed time; the derivative
resets after interruptions over 50 ms and when leaving recovery. At cruise 100,
normal steering commands 45–155 PWM; this does not establish motor stall speed.
Corner entry requires three outer-only samples
with at least 0.12 normalized strength; direction memory requires two. These
confidence levels need checking against actual sensor readings on the track.
