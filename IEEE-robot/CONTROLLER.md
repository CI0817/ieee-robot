# Using the line controller

This firmware controls forward line following and recovery using the three analog IR inputs. End-zone detection, capacitor-zone behaviour, ultrasound and ball capture are not implemented. The previous unfinished routines were removed because ordinary junctions could trigger them. There is no route memory or general loop-solving algorithm.

## First run

1. Build/upload the `main` PlatformIO environment. Open a serial monitor at **115200 baud**. The robot always boots stopped; it does not start automatically after reset, including after a battery change.
2. Send `l` to enable 10 Hz diagnostics. Confirm that GPIOs 32, 35 and 34 correspond to left, centre and right. Confirm tape produces higher readings than floor. If it produces lower readings, change `BLACK_IS_HIGH` to `false`, rebuild and recalibrate. Calibration supports different measured ranges for each sensor, but the sweep assumes the configured polarity for all three.
3. Send `c`. For **six seconds**, manually sweep the sensor bar so **every sensor** sees both actual floor and tape at the intended mounting height. Motors remain stopped. Calibration requires at least 300 ADC counts of contrast per sensor. Bad contrast should prompt checking height, wiring and material response before lowering this threshold.
4. A successful calibration is saved in ESP32 nonvolatile storage. Place the robot centred on the line, then send `g` to start. Send `s` to stop. After a recovery timeout, it remains stopped until you reposition it and send `g`.
5. With wheels lifted, confirm that a right-side line makes the left wheel run faster, and a left-side line makes the right wheel run faster. Both positive wheel commands must physically drive forward. Correct motor wiring/pin assignment if this is wrong.

Use `pio device monitor -b 115200` if your monitor defaults to another baud rate. The host must send `g` on each boot; a physical start button is not assigned because no spare button wiring was provided. Calibrate again after changing height, tape, floor or lighting. This sweep uses extrema, so electrical spikes can spoil calibration; inspect raw readings with motors running as well.

Log format: timestamp in ms, numeric mode, three raw ADC readings, normalized position error and signed wheel PWM. Modes: 0 stopped, 1 calibrating, 2 tracking, 3 gap, 4 searching, 5 corner. Logging is skipped when the transmit buffer lacks space. A position error of zero during a gap does not mean the line is centred: inspect the mode and raw readings.

## Behaviour and tuning

- Sensors are sampled once every scheduled 5 ms cycle. Each is calibrated separately; hysteresis classifies black while continuous readings estimate position. The PD derivative uses actual elapsed seconds and a 25 ms filter. Integral is disabled.
- Tune `KP` at low speed with `KD = 0`, then add a little `KD` to damp oscillation. The provided gains are starting values, not hardware-validated values. Derivative gains from controllers without a seconds-based derivative are not interchangeable.
- Base PWM decreases as lateral error increases. Measure the lowest reliable PWM under load; the slow and gap commands may need raising if the motors stall. `LEFT_TRIM` and `RIGHT_TRIM` apply only to positive commands. PWM limits are 80 in either direction.
- An outer-only detection must persist for 45 ms to enter a pivot. A centre reading prevents ordinary arcs from taking that path. A pivot ends after 30 ms of centre-black and both sides-white readings; failure to reacquire within 1200 ms stops the robot. This is a heuristic: a sufficiently off-centre curve can also require recovery.
- A gap initially scales down both wheel commands using the recent filtered correction and preceding base PWM. After 160 ms it searches toward the last reliable line side for 450 ms, then sweeps back. With no direction history it uses the branch preference. Searching must reacquire a centred line; it stops after a total loss duration of 1560 ms. These timings must be measured against real speed, gap length and turning rate. Reduced PWM only approximates reduced speed/constant curvature.
- Both outer sensors black is ambiguous, including all-black. The firmware slows and biases toward `BRANCH_PREFERENCE` (left by default). It does not label the patch as an end zone. This simple preference cannot guarantee selection of every Y branch or completion of a looped maze. A proper route/junction state machine needs course-specific evidence.

## Mounting

Use a rigid adjustable transverse row, centred on the chassis and ahead of the drive axle. Start near 2.5 mm sensor-face clearance for TCRT5000, then optimize measured tape/floor contrast. Adjust spacing by sliding across the narrowest and widest tape: avoid blind all-white intervals between sensors and avoid all three remaining fully black across a wide lateral range. Corner reacquisition assumes a centred line can produce centre-black with both sides-white. If the tape/sensor geometry cannot produce this, adjust the geometry and/or revisit that criterion before running corners.

Keep the optical centres symmetric and the sensor bar level. Measure wheel alignment, minimum motor PWM and motor mismatch. No software change substitutes for a usable optical signal.

## Verification

Firmware build:

```sh
pio run -d IEEE-robot -e main
```

Host regression tests from the repository root (no hardware required):

```sh
c++ -std=c++11 -Wall -Wextra -Werror -I IEEE-robot/test/host IEEE-robot/test/host/controller_test.cpp -o /tmp/ieee-controller-test
/tmp/ieee-controller-test
```

On Macs whose compiler cannot locate C++ headers, add `-isystem /Library/Developer/CommandLineTools/SDKs/MacOSX.sdk/usr/include/c++/v1`.

Tests check steering direction on both sides, arc versus pivot selection, corner reacquisition/timeout, brief-gap reacquisition, zero-history searching, persistent-loss stop, explicit junction preference, calibration validation and inverted normalization. They mock the hardware and do not validate motor response, actual timing, flash storage persistence or performance on a physical course.
