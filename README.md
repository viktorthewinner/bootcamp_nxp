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
| `source/intersection.c` | Spots a crossing and drives straight over it |
| `source/pixy.c` | Pixy2 line-tracking driver (rewritten — see below) |
| `source/ticks.c` | Microsecond clock on SysTick, so the loop has a real `dt` |
| `source/main.c` | Init, then a four-line loop |
| `test/` | Host simulator that runs the real control code — see *Testing* |

`track.c`, `racing_line.c`, `intersection.c` and `driver.c` have **no SDK dependencies at all**. That is
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

### 1b. Stop the camera merging curves — this is the "turns in too early" fix

Point the camera at a smooth corner and look at the arrows PixyMon draws. If a curve
comes back as **one long straight arrow**, the tracker has fitted a single line across
the whole arc. That single fact causes the car to turn in early, and no amount of gain
tuning fixes it:

- a straight line drawn across an arc touches the real line only at its two ends, and
  sits **inside** the bend everywhere in between
- both track edges sag inward together, so the corridor centre sags inward too
- worse, `headNear` and `headFar` come out equal, so `curv` is zero — the car reads a
  corner as *a straight road at an angle*, and steers to line up with it immediately.
  The corner effectively gets smeared backwards into the straight before it

In PixyMon's **Expert** tab, lower the line merging distance and the minimum line length
until a smooth corner shows **two or three separate arrows instead of one**. That is the
whole fix, and it is worth more than any tuning below.

The firmware copes either way — it detects the low vector count and both aims at the far
end of the chord (where the error returns to zero) and scales back the racing line, which
keeps the car off the lines. But it cannot invent curvature the camera threw away, so the
line stays compromised until the camera is fixed. `tools/capture.ps1` reports this
directly: it counts vectors per frame in corners and tells you if they are being merged.

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

### 2b. Two numbers for the intersection detector

Everything else in this firmware works in image columns and learns its own scale.
Intersection detection cannot, because what it looks for is **a break in a black
line about one track width long** — and that is a statement about the track, not
about the picture. The same 45 cm gap is thirty image rows deep at the bumper and
three near the horizon. So the frame is unprojected onto the ground before anything
is measured, and that needs to know where the camera is:

| Constant | How to get it |
|---|---|
| `CAM_HORIZON_ROW` | Put the car on a long straight. In PixyMon's line view, extend the two black lines until they meet. The row they meet on is this number — negative, because on any sanely aimed camera the meeting point is above the top of the frame. |
| `CAM_HEIGHT_CM` | Height of the lens above the track surface. A ruler. |

**Get these right before blaming anything else.** They scale every distance the
detector measures. With them right, everything below is free — every circuit,
chicane and camera-failure test runs identically with the feature in and out. With
them wrong, distances come out scaled and the detector starts reading ordinary
corners as crossings.

That is measured, not theoretical, and it is why one half of the detector ships
switched off:

| | Ships | Finds | Needs the two numbers above to be |
|---|---|---|---|
| **Far-side test** — the edge stops, white space, the same edge picks up again parallel and in line | **on** | a crossing from 1–2 m out | roughly right |
| **Doorstep test** — both edges stop together, nothing beyond, a bar lying across the track | **off** (`ISEC_MOUTH_ENABLE`) | the same crossing once the car is on top of it and the far side is no longer in frame | right |

The doorstep test is the one that recognises a crossing the car is already sitting
in. It is also the fragile one: "both lines stop and there is white space beyond"
is what a badly aimed camera reports in an ordinary corner too. Across the 162
mountings in the robustness sweep — where the simulated camera deliberately does
*not* match these constants — turning it on costs a dozen runs that finished,
including one inside the recommended mounting envelope. With a camera that does
match, it costs nothing at all.

So: **measure the two numbers, check them, then set `ISEC_MOUTH_ENABLE 1`.** In
that order. If you would rather not have any of it, `ISEC_ENABLE 0` puts the car
back exactly as it was.

### 3. First power-up

The wheels stay dead and straight for 1.5 s after power-on (`START_DELAY_MS`) so you can
place the car, then the speed ramps in. If a wheel spins the wrong way, set
`MOTOR1_INVERT` / `MOTOR2_INVERT` in `race_config.h`.

---

## Tuning, in the order that works

1. **`SPEED_MIN`** (default 44) is the corner floor, and on this car it matters more than
   `SPEED_MAX`. It was originally 32, which turned out to be *counterproductive*: at low
   corner speed the look-ahead row pulls in close, the line gets twitchy, and the car was
   both slower and closer to the lines. Raising it to 44 cut a fifth off every lap and
   increased clearance at the same time.
2. **`SPEED_MAX`** (default 75). The straight-line ceiling. Raise it 5 at a time until the
   car looks nervous, then back off 5. It is not a lap-time dial - corner speed is set by
   what the camera can see, not by this number.
3. **`STEER_KP` / `STEER_KH`**. Wobbling on straights → lower them. Turning in late and
   touching the outside line → raise them.
4. **`LINE_APEX_BIAS`** (0.85). How hard it dives for the apex. Lower it if the car kisses
   the inside line.
5. **`SAFE_MARGIN_FRAC`** (0.22). The safety belt — the keep-out band beside each line as a
   fraction of track width. Raising it makes the car drive more centrally and more safely.
6. **`CHICANE_DEADBAND_BIG`** / **`CORNER_HEAD_IGNORE`**. How much wiggle gets ignored.
7. **`STEER_LIMIT_RIGHT`** (45) is carried over from the original config and is noticeably
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
- the two wheels are split **around** the commanded speed through a corner - outside up,
  inside down by the same amount - which rotates the car into the turn without costing
  any drive. Slowing only the inside wheel, the obvious way to do this, quietly throws
  away up to a fifth of the thrust at full lock, exactly where the car needs it most

**Crossings.** An intersection is the one place the track model is actively wrong:
two more black lines cut across the corridor, `track.c` throws them away as start
lines, and what is left opens out sideways for a few frames — so the racing line
aims at the gap and the car turns down the crossing track. `intersection.c` gets in
first, and it looks for the hole rather than for the corners. The edge the car is
following stops, there is about one track width of white space, and then the same
edge picks up again on the far side, parallel to where it left off and in line with
it. The bars across the mouth are never used — they are thrown away with everything
else that does not run up the track.

Requiring the far piece is what makes this specific. A plain corner does not look
like this: its edges are continuous, and when one leaves the side of the picture
nothing appears beyond it. Neither does a start line, which is a bar across an
unbroken pair of edges. And the *other* edge gets a veto — a crossing cuts both
black lines at the same place, so if the line on the far side of the car runs
straight through the hole, the hole is this edge dropping out while the track
carries on.

Close up, though, the far pieces are a couple of pixels tall at the top of the
frame and the camera often stops reporting them, which leaves only both edges
stopping together with nothing beyond. That is a much weaker signature — it is also
what a camera that cannot see far reports in a corner — so it is held up by three
things: both edges must stop, within 25 cm of each other, and there must be a line
lying *across* the track at or beyond where they stopped. That is the only use made
of the crossing bars, and it is evidence, not steering. It still ships switched off
behind `ISEC_MOUTH_ENABLE`, because all three are measured through the camera
calibration; see *Before the first run*.

Then the car lines itself up **parallel to the track and drives**. Parallel, not
centred: the visible edges have a heading relative to the car, the steering nulls
it, and holding that means the car comes out of the crossing on the same line it
went in on. Aiming at a computed centre would need the corridor width, which is
exactly the thing that cannot be measured inside a crossing. How far to drive is
not a guess either — the gap was measured on the way in, so the latch runs to the
far side of it plus the length of the car. While it is crossing, the lost-track
budget is frozen, since a missing corridor there is the expected answer rather than
a failure.

Until it commits, the module changes nothing at all. That is checked, not assumed:
every simulation mode is byte-identical with it compiled in and with `ISEC_ENABLE 0`.

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

## Getting data off the car

There was no way to see what the car was doing, and both obvious routes are dead ends
on this board. The debug console is semihosted, so it needs a debugger attached and
stalls the control loop for milliseconds per line. The debug UART is worse: `board.h`
puts it on LPUART4, but **LPUART4 is not in this project's pin mux** — only CTIMER0,
CTIMER2, GPIO0, LP_FLEXCOMM0 and LP_FLEXCOMM2 are routed. That is why the MCU-Link
VCOM port is silent no matter what you print.

So instead there is a flight recorder. One 32-byte record per camera frame goes into a
ring buffer in SRAMX — a 96 KB block this project otherwise never touches — and the
whole thing is read out afterwards through the SWD probe. It costs a few dozen
nanoseconds per frame, needs no cable while the car drives, and the buffer is in the
linker's no-init section so the log survives a reset.

```powershell
.\tools\capture.ps1                  # capture and print the audit
.\tools\capture.ps1 -Csv run1.csv    # also dump every frame to CSV
```

**Stop the debugger in VS Code first** — only one thing can own the probe at a time,
and a debug session left running is the usual reason capture fails. The script checks
and tells you.

The decoder does not just dump numbers; it reports the things that cannot be known
from the code or from simulation because they depend on your camera and your track:
the real frame rate, how often each line was visible, the corridor width the car
actually measured, whether the corridor sits centred in the frame, and whether the
heading values land in the range the corner thresholds assume. Then it says what it
thinks is wrong.

### Bench mode

`RACE_BENCH_MODE` in `race_config.h` runs everything — camera, track model, racing
line, servo — with the drive motors held at zero. Use it for the first capture, with
the car in your hand over the track. The steering will move, so you can see the
firmware reacting, but nothing drives off the bench. Set it back to `0` to race.

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
| Oval, 90 cm radius | 7.22 s | +4.96 cm |
| Tight 180s, 55 / 80 cm | 5.92 s | +4.95 cm |
| Chicane + sweepers | 8.10 s | +7.23 cm |
| Mixed circuit, started off-centre | 6.02 s | +3.50 cm |
| Narrow 35 cm track | 6.54 s | +0.03 cm |

The 35 cm track is deliberately narrower than any real NXP Cup lane (45-60 cm) — the car
is 14 cm wide, so it fills 40% of it. Every realistic circuit clears by 3.5 cm or more.

**Chicanes** — the little ones are ignored, and at full speed:

| Feature | Peak steer | Speed through it | Result |
|---|---|---|---|
| Tiny wiggle (~8 cm) | 9.5 | 267 cm/s (straight-line speed) | **ignored, drove straight through** |
| Small chicane | 26.1 | 220 cm/s | mild correction |
| Medium chicane | 52.6 | 114 cm/s | steered |
| Real S bend | 60.0 | 118 cm/s | steered and braked |

**Intersections** (`./build_and_run.sh -isec`) — three separate checks:

*The scan on its own.* Fifteen cases, written in centimetres where the black lines
really sit on the track and then projected into the frame and rounded to whole
pixels the way the Pixy2 would report them. A crossing is found whether it breaks
the left edge, the right edge or both, with the car up to 15° off line, and with
the near edge arriving as a chain of three separate vectors. An unbroken edge, an
edge that simply runs out of look-ahead, a 10 cm camera dropout, a 140 cm hole, a
far piece that is not parallel, one that is 40 cm out of line, an edge with nothing
near the car, and a frame containing only the crossing bars are all rejected. The
last two cases check the steering points the right way.

*False alarms.* Five ordinary circuits with no crossing anywhere on them, ~2600
camera frames including two hairpins, a chicane and a 35 cm-wide track. **Not one
frame is even detected**, let alone committed to — with the doorstep test switched
on as well as off.

*Driving through one.* The main edges stop for the width of the crossing track and
pick up again on the far side, with four bars run out of the corners — exactly what
the camera would see. Each case runs twice, once with the crossing and once with the
same stretch left whole, because a racing line uses the width of the track on
purpose and without the control there is no telling which put the car where:

| Case | Frames detected | Committed | Peak offset, crossing / no crossing | Result |
|---|---|---|---|---|
| Straight, crossing at 3 m | 31 | 1 | 0.2 cm / 0.6 cm | straight through |
| Same, car starts 10 cm left | 34 | 1 | 6.4 cm / 6.3 cm | straight through |
| Same, car starts 10 cm right | 33 | 1 | 6.7 cm / 6.6 cm | straight through |
| Short 20 cm bar stubs | 31 | 1 | 0.2 cm / 0.6 cm | straight through |
| Crossing just after a bend | 24 | 1 | 13.8 cm / 13.3 cm | straight through |
| Far side never reported at all | 0 → 2 | 0 → 1 | — / 0.6 cm | needs `ISEC_MOUTH_ENABLE` |
| Same, car 10 cm off line | 0 | 0 | — / 6.3 cm | not recognised |

Committed exactly once each, at about 50 cm out, no line contact. The two offset
columns are the point: holding heading through the crossing leaves the car within
half a centimetre of the line it would have been on anyway. The 13.8 cm in the
"after a bend" row is the racing line running wide out of the bend, not the
crossing.

The last two rows are the camera never reporting the far side of the crossing at
all. The first needs the doorstep test switched on; the second needs it *and*
`ISEC_MOUTH_ONE_SIDED`, which is off for good reason — see race_config.h. Both
fail safe: the car drives the crossing as ordinary track, or stops in it.

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
- Intersection detection assumes **flat ground** and needs `CAM_HORIZON_ROW` and
  `CAM_HEIGHT_CM` to be roughly right (see *Before the first run*). It is the only part
  of the firmware that is not calibration-free. On a banked or humped track the
  unprojection is wrong and crossings will be missed.
- As shipped it needs to see the far side of the crossing before it will commit, so
  a crossing the car is already sitting in — where the far edges are a couple of
  pixels at the top of the frame and the camera has stopped reporting them — is not
  recognised. `ISEC_MOUTH_ENABLE` is the switch for that case and it is off by
  default; it requires the camera constants to be measured first. Either way it
  fails the right way: the car drives the crossing as ordinary track, or stops in it.
- A crossing arrived at well off line, close enough that only one black line is in
  the picture at all, is not recognised even with the doorstep test on. One line
  stopping is not distinguishable from a hairpin without a second opinion, and
  trying it (`ISEC_MOUTH_ONE_SIDED`) measurably drives the car out of corners.
- The steering holds heading, not position. The car leaves a crossing on the line it
  entered on, so if it arrives off-centre it stays off-centre until the corridor is
  believed again on the far side.
- The build is `-O0`. Moving to `-O2` is free speed in the control loop if you want it,
  but it changes timing, so re-test rather than doing it the night before a race.
- `RACE_DEBUG` must stay `0` when driving. The debug console is semihosted: with no
  debugger attached, every `PRINTF` stalls the loop.
