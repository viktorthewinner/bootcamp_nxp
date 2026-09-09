# NXP Cup — racing firmware

A rewrite of the car's control code, built for the three things you asked for:

- **drive a racing line** — outside on the way in, inside at the apex, outside on the way out
- **carry the highest speed the situation allows**, recomputed every camera frame
- **steer for corners, not for kinks** — small chicanes get driven straight through

and, above all of them, **never cross the black lines**. That is a hard constraint in
the code, not an emergent behaviour: every aim point is checked against both track
edges before it reaches the servo, and the check can override the racing line, the
chicane logic, and the speed planner.

`original code/` is untouched. This is a separate project in `race code/nxpcup_race/`.

---

## Where everything is

| File | What it does |
|---|---|
| `include/race_config.h` | **Every tunable number.** This is the file you edit at the track. |
| `source/track.c` | Pixy2 vectors → where the two black lines are, at 8 distances ahead |
| `source/racing_line.c` | Picks the point to aim at, then forces it back inside the lines |
| `source/driver.c` | Steering controller + speed planner + brakes + torque vectoring |
| `source/pixy.c` | Pixy2 line-tracking driver (rewritten — see below) |
| `source/ticks.c` | Microsecond clock on SysTick, so the loop has a real `dt` |
| `source/main.c` | Init, then a four-line loop |
| `test/` | Host simulator that runs the real control code — see *Testing* |

`track.c`, `racing_line.c` and `driver.c` have **no SDK dependencies at all**. That is
deliberate: it is what lets the whole perception and planning chain be compiled and
raced on a PC before it ever touches the car.

---

## Build and flash

Same as before — the project builds with the MCUXpresso CMake presets:

```bash
cmake --preset Debug && cmake --build --preset Debug
```

The Eclipse project name inside is still `nxpcup_official`, so the linker scripts and
launch config carry over unchanged. Only the folder name is different.

---

## Before the first run

### 1. PixyMon settings

Your current settings, and what I would change:

| Setting | You have | Suggested | Why |
|---|---|---|---|
| White line | unchecked | **keep unchecked** | Correct — the track edges are black lines. |
| Edge threshold | 100 | keep, lower if it loses the line in dim light | High contrast tape on a white track copes fine at 100. |
| Maximum line width | 40 | **15–20** | This is in grid pixels, and the grid is only 79 wide. At 40 the tracker will accept anything up to half the frame as a "line" — a shadow, a dark mat edge, a bag on the floor. Real tape is about 6 px close up and 1–2 px far away. Tightening this is the single best thing you can do to stop the car chasing dark blobs. |
| Minimum line width | 0 | **1** | Rejects single-pixel sensor noise. |
| Camera brightness | 159 | **try 90–120** | Brightness raises exposure time, and exposure time is motion blur. A fast car with a blurry frame sees short, wrong vectors. Lower it until the picture looks slightly dark but the lines are still crisp while the car is *moving*. |

Then **save the settings to the Pixy** (not just to PixyMon) and make sure the Pixy2 is
running the **line tracking** program, so it boots into it with no PC attached.

### 2. Camera aim — this one matters more than any gain

The most useful thing that came out of testing: across 162 simulated camera mountings,
every failure traced back to how the camera was aimed, not to the tuning. The rule that
separated "flawless" from "off the track" is simple, and you can check it in PixyMon in
about ten seconds:

> Put the car on a straight. In the PixyMon picture, **both black lines must be visible
> in the middle of the frame**, and the bottom of the frame should be roughly 15–35 cm
> in front of the bumper with the top around 1–2.5 m ahead.

If the lines leave the sides of the picture before the middle row, the camera is too low
or aimed too far down. Raise it or tilt it up. Within that envelope the car finished
**18 out of 18** test configurations with zero line contact. Outside it, the firmware
still fails safe — it slows down and stops rather than guessing — but it is slow.

### 3. First power-up

The wheels stay dead and straight for 1.5 s after power-on (`START_DELAY_MS`) so you can
place the car, then the speed ramps in. If a wheel spins the wrong way, set
`MOTOR1_INVERT` / `MOTOR2_INVERT` in `race_config.h`.

---

## Tuning, in the order that works

1. **`SPEED_MAX`** (default 65). The headline number. Raise it 5 at a time until the car
   looks nervous, then back off 5. Worth knowing: in simulation, raising `SPEED_MAX` from
   45 to 110 changed total lap time by only 14%, because corner speed is set by what the
   camera can see, not by this number. It is a straight-line ceiling, not a lap-time dial.
2. **`STEER_KP` / `STEER_KH`**. Wobbling on straights → lower them. Turning in late and
   touching the outside line → raise them.
3. **`LINE_APEX_BIAS`** (0.85). How hard it dives for the apex. Lower it if the car kisses
   the inside line.
4. **`SAFE_MARGIN_FRAC`** (0.22). The safety belt — the keep-out band beside each line as a
   fraction of track width. Raising it makes the car drive more centrally and more safely.
5. **`CHICANE_DEADBAND_BIG`** / **`CORNER_HEAD_IGNORE`**. How much wiggle gets ignored.
6. **`STEER_LIMIT_RIGHT`** (45) is carried over from the original config and is noticeably
   tighter than `STEER_LIMIT_LEFT` (-60). If the car understeers in right-handers but not
   left-handers, this is why — raise it toward 60 if the linkage allows.

---

## How it drives

**Where the track is.** Each frame, the Pixy2 line vectors are cleaned up (anything close
to horizontal is a start line or an intersection bar and is thrown away), then the corridor
is walked outward from the bumper, one sample row at a time, taking the innermost candidate
on each side. Two details earn their keep:

- The camera's ~60° sideways view is narrower than the track close up, so near the bumper
  **both lines are usually out of frame even though they are plainly still there**. Lines
  are extended downward to fill those rows — cheap and safe, because the bottom of the
  frame is only the first 20 cm of track and no line bends much in 20 cm. Extending lines
  *away* from the car is kept tight, because near the horizon a couple of rows is a long
  way down the road.
- Corridor width on screen is a straight line in image row (twice as far = half as wide),
  so the car fits `width = a·row + b` to whatever rows saw both lines. **One row that can
  see both lines calibrates every row that cannot** — which is what makes the near rows
  usable at all.

**Where to aim.** The corner phase is worked out from two headings: `headNear`, the road
right in front, and `headFar`, the road at the limit of vision.

```
straight here, corner in the distance  ->  ENTRY  ->  stay out wide
turning here and still turning ahead   ->  APEX   ->  hug the inside
turning here but straight ahead        ->  EXIT   ->  let it run wide
```

These blend continuously rather than switching, so the aim point sweeps across the track
instead of jumping. Measured through a 170 cm-diameter left-hander on a 45 cm track:

```
approach   -0.6 cm   (outside)
turn in    +2.6 cm
apex       +8.4 cm   (inside)
exit      -10.5 cm   (outside)
```

**The safety check.** Whatever the racing line asks for, the aim point is then run through
a geometric solve: the car is assumed to sweep from where it is to the aim point, and every
sample row in between turns into a pair of bounds on where that aim point may be. Keeping
the tightest pair gives the exact window of aim points whose whole path stays inside the
black lines. That solve quietly overrules an over-enthusiastic apex, and cancels a chicane
that turned out to be too tight to ignore.

**How fast.** The strongest of three cues sets the speed: the far heading, the curvature,
and how hard the car is already steering. Because one of them looks at the *far* end of the
image, the car lifts and brakes **before** the corner rather than in the middle of it. Then:

- speed is scaled by how far ahead the model is still valid — you may only go as fast as
  you can see, and a camera that suddenly sees less immediately gets a slower car
- capped when only one edge is visible, because the other one is a guess
- ramped on acceleration, dropped instantly on deceleration, with a short reverse pulse
  from the H-bridge when a lot of speed has to go quickly
- the inside wheel is slowed through a corner, which rotates the car into the turn instead
  of pushing it wide

**Ignoring the small stuff.** The steering deadband is not fixed — it slides from wide to
narrow depending on how corner-like the road is. Crucially, corner-ness is judged from
`headFar` and from `headFar − headNear`, and **not** from `headNear` alone. Geometry says
why: a camera at height `h` sees a lateral position error `e` as a near-field slope of
about `e/h` whatever the road is doing, so `headNear` mostly reports *"the car is off to
one side"*. Reading corners off it makes the car brake for its own untidiness — which is
exactly what the first version did.

Ignoring a kink is safe because the safety check still owns the black lines: the moment
driving straight would put the car near one, the deadband drops to zero and it steers. So
a long gentle curve is not ignored forever — the car drifts across the track, using its
width, and corrects only when it has to.

---

## What changed in the Pixy2 driver

The old driver worked, but four things about it were dangerous at speed:

1. It read a fixed 100 bytes every frame. At 100 kHz that is **9 ms per frame** and capped
   the control loop near 100 Hz. It now reads the 6-byte header first and then only the
   bytes that actually exist — typically 3 ms.
2. It never checked the sync word, the packet type, or the payload checksum, and parsed
   whatever happened to be in the buffer. All three are verified now, and the driver
   resynchronises when the bus gets out of step.
3. A "busy" reply — the camera has no new frame yet, which is normal whenever the loop runs
   faster than the camera — was being decoded as vectors. It is now reported as *no new
   data*, and the driver keeps the previous frame.
4. `while (!transferDone) {}` had no timeout. One stuck I2C transaction meant a moving car
   with no steering and no braking, forever. Every transfer now times out, aborts, and
   feeds the failsafe.

Other fixes worth knowing about: `HbridgeInit` copied the uninitialised struct over the
global before filling it in; `Steer()` and the control path used `double`, which is
software-emulated on this part (the FPU is single precision only); and motor duty was
limited to whole percent, which is a visible stutter when the planner is trimming the
throttle through a corner.

---

## Testing

The control code is compiled and raced on the PC before it goes near the car:

```bash
cd test && ./build_and_run.sh
```

This builds `track.c`, `racing_line.c` and `driver.c` **unmodified** — the same files that
run on the MCU — against a bicycle-model car with a grip limit, a 60 Hz camera, a 250 Hz
control loop, and a perspective-projected 79×52 frame that is integer-quantised and loses
the track edges out of the sides of the picture exactly like the real one.

Results with the shipped defaults:

**Circuits** — all five finish, none touches a line:

| Circuit | Lap | Closest approach to a line |
|---|---|---|
| Oval, 90 cm radius | 9.35 s | +2.44 cm |
| Tight 180s, 55 / 80 cm | 7.91 s | +2.19 cm |
| Chicane + sweepers | 10.36 s | +7.15 cm |
| Mixed circuit, started off-centre | 7.82 s | +3.50 cm |
| Narrow 35 cm track | 8.00 s | +0.17 cm |

**Chicanes** — the little ones are ignored, and at full speed:

| Feature | Peak steer | Speed through it | Result |
|---|---|---|---|
| Tiny wiggle (~8 cm) | 8.8 | 229 cm/s (straight-line speed) | **ignored, drove straight through** |
| Small chicane | 23.7 | 176 cm/s | mild correction |
| Medium chicane | 45.0 | 121 cm/s | steered |
| Real S bend | 60.0 | 87 cm/s | steered and braked |

**Camera failure** — **no line contact in any of these**:

| Fault | Result |
|---|---|
| 20% / 50% / 80% of frames lost | stays inside the lines, finishes |
| Blind for 20 cm entering a corner | stays inside the lines, finishes |
| Blind for 60 cm entering a corner | stays inside the lines, **stops** |
| Blind for 150 cm through a corner | stays inside the lines, **stops** |
| 40% lost + blind for 40 cm | stays inside the lines, finishes |

The two that stop do so on purpose. Past a certain distance everything the car does is
guesswork, and the only thing bounding how wrong that guess gets is how far it travels,
so it sheds speed and stops — inside the lines — rather than driving on blind. Losing
frames is handled differently: the car keeps racing, but scaled down in proportion to how
far the frame rate has fallen, because a correction that arrives twice as late buys half
the safety margin.

**Mounting robustness** — 162 combinations of focal length, camera height, tilt, track width
and corner radius. Inside the aiming envelope described above: **18/18 finished, 18/18 with
zero line contact.** Outside it: 124/144 still clean, and most of the rest stop safely
rather than crash.

Simulated numbers are not lap times — the mapping from a speed command to cm/s depends on
your motors and battery, and I had to assume one. What the simulation does establish is
that the *logic* is sound, that the safety constraint holds under fault and across
mountings, and that the chicane and racing-line behaviour is real rather than hoped for.

---

## Known limitations

- **`STEER_LIMIT_RIGHT = 45` vs `STEER_LIMIT_LEFT = -60`** is inherited from the original
  config and I could not verify it against the hardware. If it is a tuning choice rather
  than a mechanical stop, the car is giving away right-hand corners for nothing.
- A corner tighter than the car's turning circle cannot be driven at any speed. With the
  limits above that is roughly 70 cm radius to the right and 52 cm to the left. The
  firmware fails safe there — it slows and, if it truly cannot see a way through, stops.
- The build is `-O0`. Moving to `-O2` is free speed in the control loop if you want it,
  but it changes timing, so re-test rather than doing it the night before a race.
- `RACE_DEBUG` must stay `0` when driving. The debug console is semihosted: with no
  debugger attached, every `PRINTF` stalls the loop.
