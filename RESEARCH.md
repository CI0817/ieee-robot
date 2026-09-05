# Line-following and trap-handling research

Research date: 5 September 2026. Hardware scope: two TCRT5000 reflectance sensors, RCWL-1601 ultrasound, ESP-WROOM-32, TB6612FNG, two N20 100:1 100 rpm gearmotors, AA NiMH batteries and LM2596. No wheel encoders, IMU, colour sensor or capture actuator are listed. This is a design recommendation based on source review, not a tested implementation on this robot.

**Recommendation.** Build a calibrated analog P/PD line controller, a finite state machine for ambiguous track events, and a route history for reverse retracing. Use ultrasound for ball approach only after identifying the end box. Treat loop recognition as uncertain unless the practice/trial course provides enough distinctive structure. I found useful implementations, but no verified public implementation that demonstrates this exact hardware completing all the required traps and ball retrieval. “Best” below means best-supported starting points for this task, not a benchmark winner.

**The rules materially change the design.** The linked rules specify 9–18 mm black PVC tape, variable layouts with gaps and intersections, a larger black end box, a smaller start box, and a ball in the middle of the end box. They require autonomous collection followed by backwards travel through the track used outbound. The complete robot and ball must stop inside the start box. The field is otherwise open. Hardware substitutions must match the supplied model/specification. There is one seven-minute final-track trial and one seven-minute competition attempt; checkpoint progress takes priority, with adjusted time resolving ties. [Competition rules supplied in README](https://docs.google.com/document/d/1g6Eb3Ngv5g4ZNUFfK-9r6EEk7T_82anK/edit).

The rules do not give a maximum gap, minimum bend radius, junction angles/separation, box dimensions, or a guarantee of a regular grid. They also combine gaps with requirements to remain over tape and keep the frame covering the track. Ask organisers to define legal gap crossing and recovery movement. Confirm whether “backwards” requires reverse motor travel without turning the robot around, whether return must repeat exploratory detours, and whether learned data/program changes between trial and competition are allowed. These are engineering dependencies, not reasons to postpone basic sensor and drive testing.

**Existing implementations worth studying.**

| Reference | What is implemented | Fit and limitation |
| --- | --- | --- |
| [Pololu 3pi line-maze solver](https://www.pololu.com/docs/0J21/all), with [AVR source repository](https://github.com/pololu/libpololu-avr) | Separate segment following, junction detection, turning, route recording and replay | Strongest baseline for software structure. Uses five reflectance sensors and different hardware; its basic maze strategy explicitly assumes no loops. Port the structure, not sensor indexes, gains or timings. |
| [Pololu engineer's looped-maze implementation](https://forum.pololu.com/t/3pi-looped-maze-solver/3269/1) | Posted C code with visited cells, connectivity, flood distances and route generation | Strongest concrete loop-handling reference found. Assumes a six-inch grid, up to 16×16 grid points, and heavily calibrated travel timing. The author explicitly says arbitrary courses need modification. Your rules do not guarantee that grid. |
| [LineMaster Pro, March 2026 preprint](https://arxiv.org/html/2603.13907v1) | Two TCRT5000s, HC-SR04, PID pseudocode and obstacle/recovery states | Closest recent sensor-count match. Different ultrasonic module, Nano, L298N and motors. No demonstrated looped-maze exploration, ball capture or reverse retracing. Useful as a comparison, not ready competition firmware. |

The 2026 paper reports 1.18 cm mean tracking error at 0.4 m/s. That error is comparable to or greater than the half-width of your tape; it is not evidence of adequate performance here. Its controller reduces analog readings to binary values, and its spiral recovery leaves the line. I did not locate downloadable firmware or raw trial data in the inspected paper. Its gains and reported success rates should not be inherited as verified targets. [Paper, methods and results](https://arxiv.org/html/2603.13907v1).

Pololu's PID example is useful for understanding steering correction and motor saturation. Your calibration, sample period, error units and drivetrain will differ, so its numerical gains do not transfer. [Pololu PID implementation](https://www.pololu.com/docs/0J21/7.c).

**What two sensors can actually tell you.** A single observation cannot reliably classify track topology. Both white can mean a gap, a dead end, a missed bend, or a narrow line passing between widely spaced sensors. Both black can mean ordinary wide tape, a crossing, or a box. A left-only detection can be a lateral error, curve or branch. Ultrasound measures raised objects; it does not reveal the connectivity of flat tape.

Resolve ambiguity through time and small controlled motions. Retain raw readings, black/white confidence, last reliable steering direction, recent motor commands and elapsed time. The state machine must distinguish “unknown” from “dead end” and “candidate junction” from “confirmed junction.” No PID tuning eliminates these ambiguities.

**Sensor mounting and calibration come first.** The TCRT5000 is an IR emitter plus phototransistor; a module may expose analog output, comparator output, or both. Identify the supplied board before designing the interface. Vishay specifies peak operating distance at 2.5 mm; use that as the initial mounting height, then test actual PVC tape, floor finish, lighting and chassis movement. [Vishay datasheet](https://www.vishay.com/docs/83760/tcrt5000.pdf).

Use an adjustable lateral and longitudinal mount. Try to keep at least one optical footprint overlapping the narrowest tape during centred travel. Measure optical detection positions, not module PCB spacing. If the package prevents suitable spacing, an edge-following arrangement is an alternative: one sensor regulates a tape edge while the other supplies context. That sacrifices symmetric junction information and also needs testing.

A spacing greater than 9 mm can leave a blind corridor over the narrowest tape. A close spacing helps narrow-line presence detection but puts both sensors entirely on 18 mm tape, reducing lateral error information. Two sensors cannot provide both the coverage and position resolution of a wider array across all widths. Test both extremes before fixing the chassis.

For each sensor, measure white and black values independently, including with motors running. Normalize:

```text
blackness_i = clamp((raw_i - white_i) / (black_i - white_i), 0, 1)
```

Reject calibration when black and white are insufficiently separated. This formula handles either electrical polarity. Use separate hysteretic thresholds to classify black/white, while retaining analog blackness for steering. Freeze calibration during gaps and boxes so these events do not corrupt it.

When the line overlaps the sensors and neither reading is saturated, start with `e = blackness_right - blackness_left`, then `u = Kp*e + Kd*filtered_de_dt`. For forward travel, positive error should steer right: left motor faster, right motor slower. Verify motor polarity physically. This is a local error proxy, not a global line-position estimate; both-dark and both-light states require context. Begin with P, add modest filtered D only if it improves measured oscillation, and initially leave integral disabled. Treat persistent motor mismatch with per-motor trim. If only digital outputs are available, use a hysteretic discrete controller and lower speed rather than assuming analog precision.

**Proposed trap-handling behaviour.** The following is an engineering synthesis to prototype, not behaviour established by the cited robots on your course.

| Situation | Recommended response | Important limit |
| --- | --- | --- |
| Sudden gap | Debounce loss; if preceding motion was reliably straight, make a short slow forward probe with limited steering; reacquire tape before resuming normal speed | Time-limit the probe using measured speed and allowed gap size. Arbitrarily long gaps cannot be distinguished from a dead end. |
| Loss during a bend | Slow and search first toward the last reliable line direction, within the permitted footprint | Continuing straight is inappropriate when the previous evidence indicates curvature. |
| Fake split-off | Slow, record the candidate, inspect continuation, and prefer a verified known route; for unknown routes, straight-first is a useful initial heuristic | A branch cannot be identified as “fake” from its entrance alone. A real solution may require turning. |
| Junction | Collect an observation sequence using short advance/pivot motions; record available exits; execute the selected exit and confirm reacquisition | With two sensors, exits are sensed sequentially and may be missed. Closely spaced junctions and acute forks are especially difficult. |
| Dead end | Classify only after bounded gap/bend recovery fails; retrace to the preceding decision and try another exit | Keep the route record aligned with actual backtracking. A white reading alone is insufficient. |
| Loop | Use confidently recognized junctions and visited edges to avoid repeated exploration; use repeated event sequences only as a warning | Similar junctions and timing drift can produce false loop matches. Never silently merge nodes from one matching pattern. |
| Broad black region | Enter a box-candidate state; check duration/extent and expected route context while moving slowly | Both black alone also occurs on wide tape and intersections. Exact box and intersection bounds are needed for a reliable discriminator. |

During a gap, avoid preserving a large stale differential command: it can turn the robot away from a straight continuation. A short heading-hold approximation is possible after straight travel, but without encoders/IMU it is open-loop. For turns, use a calibrated minimum motion to clear the incoming line, then sensor-confirmed acquisition of the selected exit, with a maximum timeout. Stop on failed acquisition. Fixed-time turns alone accumulate error.

Pololu's junction code likewise collects evidence while advancing before selecting a turn; its array makes left/straight/right discrimination easier than it will be here. This is a useful pattern to adapt, but its delays and end-marker test depend on its course. [Pololu main maze loop](https://www.pololu.com/docs/0J21/8.d).

**Choosing the navigation algorithm.**

| Algorithm | Recommendation for this robot |
| --- | --- |
| Always left/right | Useful baseline on an explicitly loop-free maze. Insufficient for arbitrary loops. |
| Straight-first with remembered alternatives | Practical initial exploration policy that avoids unnecessary turns; has no intrinsic loop guarantee. |
| DFS / Trémaux-style edge exploration | Preferred general graph strategy if junction identities and exits can be recognized reliably. Store visited edges and backtrack when no unexplored exit remains. |
| Flood fill / BFS | Useful once a reliable discrete map exists; BFS minimizes edge count, not necessarily travel time. Do not assume a grid from the board's rectangular shape. |
| Dijkstra | Useful for selecting an outbound route on a learned graph with measured travel-time costs, if rules permit it. Not a substitute for recognizing the map. |
| PID, fuzzy logic or learned steering | Addresses motion control. Does not by itself supply missing branch identity, graph memory or gap semantics. |

For a loop-free course, Pololu's assumption is explicit, and its simple strategy is well motivated. [Pololu maze assumptions](https://www.pololu.com/docs/0J21/8.a). For general graphs, the difficulty is recognizing places: research on exploration without accurate positional sensing uses extended sequences of local signatures, and notes environments where additional information is required. [Dudek, Freedman and Hadjres, 1996](https://onlinelibrary.wiley.com/doi/abs/10.1002/%28SICI%291097-4563%28199608%2913%3A8%3C539%3A%3AAID-ROB5%3E3.0.CO%3B2-O).

On your hardware, approximate signatures could combine observed exits, a sequence of previous junctions, turn direction and broad segment-time ranges. Timing is not odometry: battery state, slip, curves and ball load change it. Keep ambiguous node matches as hypotheses. A repeated sequence can trigger slower inspection or a different branch policy, but this remains a heuristic and has no guaranteed seven-minute completion. The listed sensors do not support a defensible claim of universal trap avoidance on arbitrary layouts.

**The return journey needs its own design.** Preserve an append-only record of actual movement events separately from any simplified navigation map. Store incoming/outgoing ports, selected turn, travel direction, segment timing range, observed signature and confidence. Include exploratory detours and recovery/backtracking events until the return-route interpretation is clarified.

Pololu's `simplify_path()` removes detours for faster subsequent runs. That is useful for a permitted learned outbound run, but deleting the only record of those detours could make exact return reconstruction impossible. [Pololu path simplification](https://www.pololu.com/docs/0J21/8.e).

Literal reversing is not “turn around, reverse the turn list and swap left/right.” Define left/right in a fixed robot body frame and store actual connection/motion geometry. Ideal kinematic reversal of a recorded wheel-velocity segment negates both wheel velocities and replays segments in reverse order; measured PWM and duration are not accurate wheel velocities. Use recorded actions as feedforward and resynchronize against tape events, rather than blindly replaying PWM.

Front-mounted sensors become trailing sensors in reverse. This changes the dynamics and reduces warning of corners. Merely changing steering sign or reusing forward PID gains is not a validated reverse controller. Prototype low-speed reverse following on curves early, with separate gains and reacquisition rules. Test an adjustable position nearer the drive axle as a compromise; at the axle, heading information is also weak. If reverse tracking fails, resolve placement/mechanical sensing options within the supplied-parts rules before investing heavily in maze software.

**Ultrasound and ball capture.** Keep object-search decisions disabled until the end box is confirmed. The RCWL-1601 sold by Adafruit supports 3–5.5 V operation and HC-SR04-compatible signalling; its table lists a typical 50 ms measurement cycle and a 2–3 cm blind region. Its long-range specification is for a flat wall, not a ping-pong ball. [RCWL-1601 product documentation](https://www.adafruit.com/product/4007).

Mount and test against the actual ball at relevant heights, distances and approach angles. A small curved target can produce intermittent echoes. Treat a timeout as unknown, never as zero distance or proof of capture. Confirm approach using several valid readings, creep near the capture zone, and ensure the mechanism engages before the blind region makes distance unreliable. Any scan must remain within the permitted end-box movement and avoid pushing the ball outside.

The README contains no gripper or servo. A passive funnel and retaining flap/pocket is a possible mechanical prototype only if the provided materials permit it. A scoop that merely pushes the ball is insufficient for reversing with it. Ultrasound alone cannot reliably prove secure retention. Ball acquisition, retention through reverse curves, and full-body stopping inside the small start box require physical tests and measured box dimensions.

**ESP32 and drivetrain implementation notes.**

- Prefer analog-capable inputs such as GPIO34/35 if exposed on the actual board; both map to ADC1 and are input-only. Verify module output levels before connecting to 3.3 V logic. ESP-WROOM-32 identifies a module, not the surrounding development-board regulator or pinout. [Espressif module datasheet](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.html).
- Run TB6612 logic at a compatible voltage; verify each motor's rated voltage and stall current. N20 identifies a motor form factor, not its electrical load. Toshiba's 3.2 A figure is a short single-pulse rating, not continuous capacity. [Toshiba datasheet](https://toshiba.semicon-storage.com/info/datasheet_en_20141001.pdf?did=10660).
- Count the NiMH cells and test supply voltage under motor startup. LM2596 is a step-down regulator and cannot maintain an output above a depleted pack; input headroom matters. [TI LM2596 documentation](https://www.ti.com/product/LM2596).
- Suggested starting rates, to validate: 200 Hz line sensing/control, 50–100 Hz state-machine processing and at most roughly 20 Hz sonar ranging. A 5 ms control period moves 0.5 mm at 0.1 m/s. Sonar timeouts, serial logging and flash writes must not block that control loop.
- Use timestamped nonblocking motion states and bounded timeouts. Log raw readings, state changes, signed motor commands and route events. Store high-rate logs in RAM; persist calibration and learned route only at deliberate points.
- Start around 0.05–0.10 m/s if the drivetrain can run smoothly there. This is a test proposal, not a guaranteed achievable setting. Estimate free wheel speed from `v = pi * wheel_diameter * rpm / 60`; measure loaded speed rather than relying on the 100 rpm label.

Suggested firmware states: `CALIBRATE`, `FOLLOW_FORWARD`, `LOSS_PROBE`, `JUNCTION_PROBE`, `TAKE_EXIT`, `BACKTRACK`, `END_BOX_CONFIRM`, `BALL_APPROACH`, `CAPTURE`, `FOLLOW_REVERSE`, `START_BOX_CONFIRM`, `DONE`, and `FAULT_STOP`. Each moving state needs explicit success evidence, a time/motion limit and a failure transition. Freeze/reset controller history as appropriate when changing between following and discrete manoeuvres.

**Build and validation order.**

1. Measure analog response over 9 mm and 18 mm tape; fix height, spacing, polarity and noise before controller tuning.
2. Demonstrate forward and literal reverse tracking on straights, both bend directions and S-curves. Tune separately, then repeat with the ball retained.
3. Test straight gaps of increasing length, gaps after bends, and real dead ends that initially produce identical readings. Establish a documented recovery envelope.
4. Test T, X and acute Y junctions, short fake branches, consecutive junctions, and width transitions. Count missed and duplicate junction events.
5. Test loops with an exit, a loop returning to the same junction from another direction, and two different junctions with similar signatures. Verify ambiguous matching cannot corrupt route history.
6. Test black-box recognition versus ordinary broad tape/crossings, ball collection without displacement outside the box, reverse route reproduction, and complete stopping inside the start box.
7. Repeat at different battery levels, lighting and initial offsets. Use at least 20 repetitions per critical feature as an initial engineering screen, then full mixed-course runs; this is a proposed test plan, not statistical proof of reliability.

Record completion rate, time, recovery count, route-history correctness, ball loss and any body/tape rule violation. Increase speed only after reliable completion. Use the final-track trial to validate event signatures and difficult return segments; save a route only if permitted and actually confirmed. Because the first manual adjustment costs five seconds while later ones cost more, a long uncertain recovery is not automatically better than intervention. Optimize checkpoint progress and reliable return before shaving milliseconds off steering.

**Decisions to settle before final assembly:** exact IR module and analog-output availability; wheel diameter and motor voltage/stall current; battery count and ESP32 board variant; permissible capture materials; gap/bend/junction bounds; start/end-box dimensions; literal-reverse and detour-retracing interpretation; and trial-data retention rules. None of these details should be silently replaced with assumptions taken from a tutorial.
