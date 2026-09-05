# Ball capture

Simple ESP32 Arduino functions, independent of the line-following code:

- `setup_capture()` sets up the sensor, commands the hand to 90 degrees (release), and
  waits 500 ms for startup movement. Call once in setup.
- `read_distance_cm()` converts NewPing's `ping()` echo time to fractional
  centimeters, or returns `-1` for timeout/out-of-range. Decimal readings do
  not imply sub-centimeter accuracy.
- `ball_detected()` takes a new reading and checks `BALL_DISTANCE_CM`.
  Use `ball_detected(15.0f)` to choose a different threshold.
- `lower_hand()` starts gradual lowering; repeated calls do not restart it.
- `update_hand()` advances one degree every `HAND_STEP_INTERVAL_MS` (15 ms).
  Call frequently in loop: 90 to 0 degrees takes about 1.35 seconds.
- `hand_is_lowered()` reports when the final angle has been commanded, without
  physical feedback. It does not confirm that the ball was caught.
- `raise_hand()` cancels lowering and commands the ready position immediately.

The signal pins are echo GPIO 12, trigger GPIO 13, and servo GPIO 14.
Power the RCWL-1601 from 3.3 V for ESP32-compatible echo levels. Power the
MG90S from a suitable external servo supply (manufacturer specifies 4.8 V),
and connect sensor, servo supply, and ESP32 grounds together.
See the [sensor specifications](https://www.adafruit.com/product/4007) and
[servo specifications](https://towerpro.com.tw/product/mg90s-3/).

From the `IEEE-robot` project folder, build/upload the standalone test:

```sh
pio run -e capture_test -t upload
pio device monitor -e capture_test
```

The test commands 90 degrees at startup and gradually moves to 0 degrees once an
object is in range. These are commanded positions, without position feedback;
the 500 ms startup delay may need tuning for the loaded mechanism.
Reset to repeat. Adjust `BALL_DISTANCE_CM`, `HAND_RAISED_ANGLE`, and
`HAND_LOWERED_ANGLE` in `BallCapture.h` to fit the mechanism. Start with the
linkage disconnected to check direction and travel before attaching the hand.
The angle values map to a conservative 1000–2000 us pulse range; actual travel
depends on the servo. Lowering is a command, not confirmation of a caught ball.

The ball was observed to lose detection near 3.6 cm. Keep the initial trigger
at 4 cm and test whether that leaves enough reliable detection margin; increase
`BALL_DISTANCE_CM` if needed for the hand geometry. Do not wait for a reading
below 3.6 cm to start capturing. A missing echo does not mean the ball is caught.

For later integration, include `<BallCapture.h>` in main.cpp and call
`setup_capture()` in setup. After the end block has been confirmed and the
drive motors have stopped, use this in the capture phase (`capture_started`
is a persistent bool initialized to false):

```cpp
update_hand();
if (!capture_started && ball_detected())
{
  capture_started = true;
  lower_hand();
}
```

Distance reads deliberately wait 60 ms between pings and can take up to about
120 ms including NewPing's start/echo timeouts. Use them in the stopped capture phase, not in the
10 ms line-following loop. Servo PWM uses channel 2, leaving motor channels
0 and 1 on their existing timer. PlatformIO automatically installs
[NewPing 1.9.7](https://registry.platformio.org/libraries/teckel12/NewPing)
from the shared `lib_deps` entry in `platformio.ini`.

The test stops pinging once capture begins so sensor waits do not interrupt
the servo steps. A future capture phase can drive forward slowly while calling
`update_hand()`, then stop at a separately tuned time/distance limit. Test
stationary lowering first; forward motion has not been added to this test.

Ultrasound detects nearby objects, not ball identity or the black region.
Aim the sensor at ball height and test the small curved ball at the intended
capture distance; floor/wall echoes can also trigger detection. Readings below
3 cm are rejected because they are in the sensor's blind region. End-block
detection and integration into main.cpp are intentionally left for later.
