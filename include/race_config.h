/*
 * race_config.h - every tunable number for the racing firmware, in one place.
 *
 * Tuning order that works on a real track:
 *   1. SPEED_MAX      - raise 5 at a time until the car looks nervous, then back off 5.
 *   2. STEER_KP / KH  - if the car wobbles on straights, lower them. If it turns in
 *                       late and touches the outside line, raise them.
 *   3. LINE_APEX_BIAS - how hard it dives for the apex. Lower it if it kisses the inside line.
 *   4. SAFE_MARGIN_*  - the safety belt. Raising it makes the car drive more centrally.
 *
 * Everything is float. The MCXN947 FPU is single precision only, so never use double
 * in the control path - it turns into slow software emulation.
 */
#ifndef RACE_CONFIG_H
#define RACE_CONFIG_H

/* =====================================================================
 * DEBUG
 * ===================================================================*/
/* Keep this 0 when the car drives on its own. The debug console is semihosted:
 * with no debugger attached every PRINTF stalls the control loop. Use the flight
 * recorder below instead - it costs nothing and works while the car is untethered. */
#define RACE_DEBUG                 0

/* Flight recorder. One 32 byte record per camera frame into a ring buffer in SRAMX,
 * read out afterwards over the debug probe with tools/capture.ps1. Roughly 50 seconds
 * of history. Leave it on: it is a few dozen nanoseconds per frame and it is the only
 * way to find out what the car actually did. */
#define RACE_TELEMETRY             1

/* Bench mode: everything runs - camera, track model, racing line, servo - but the
 * drive motors are held at zero. Use it for the first capture, with the car in your
 * hand over the track, to check what the camera and the track model are really doing
 * before anything moves. Set back to 0 to race.
 *
 * The host simulator forces this to 0 on its own command line - a bench-mode car
 * never moves, so leaving it on would quietly turn every test green. */
#ifndef RACE_BENCH_MODE
#define RACE_BENCH_MODE            0
#endif

/* =====================================================================
 * CAMERA - Pixy2 line-tracking grid
 * The line tracker reports on a 79 x 52 grid.
 *   x = 0 far left ... 78 far right
 *   y = 0 far ahead ... 51 closest to the bumper
 * ===================================================================*/
#define PIXY_LINE_W                79
#define PIXY_LINE_H                52

/* Image column the car actually drives along. 39 = dead centre. Shift it if the
 * camera is not perfectly aligned: drive slowly down a straight, and if the car
 * settles left of centre, lower this number. */
#define CAM_CENTER_X               39.0f

/* ---------------------------------------------------------------------
 * Where the camera is.
 *
 * The rest of this firmware is deliberately calibration free - it works in image
 * columns and lets the track model learn its own scale. Intersection detection is
 * the one part that cannot. What it looks for is a break in a black line about one
 * track width long, and that is a statement about the track: the same gap is
 * thirty image rows deep at the bumper and three near the horizon. So the frame is
 * unprojected onto the ground before anything is measured, and these two numbers
 * are what makes that possible.
 *
 * Getting these two right is ten minutes with a ruler:
 *
 *   CAM_HORIZON_ROW  put the car on a long straight and look at the two black
 *                    lines in the line-tracker view. Extend them until they meet.
 *                    The row they meet on is this number - negative means the
 *                    meeting point is above the top of the frame, which it will be
 *                    on any sanely aimed camera.
 *
 *   CAM_HEIGHT_CM    the height of the lens above the track surface.
 *
 * Neither has to be exact. Three rows of error in the horizon changes a measured
 * gap by a few centimetres, well inside the window ISEC_GAP_MIN_CM to
 * ISEC_GAP_MAX_CM allows. Getting them badly wrong makes the detector miss
 * crossings rather than invent them, which is the right way round to fail.
 *
 * If crossings are being missed on the real car, check these two before touching
 * anything in the intersection block: a horizon that is several rows out scales
 * every distance the detector measures.
 * -------------------------------------------------------------------*/
#define CAM_HEIGHT_CM              18.0f
#define CAM_HORIZON_ROW            (-4.0f)

/* Pixy2 I2C address (default 0x54). */
#define PIXY_I2C_ADDR              0x54U

/* =====================================================================
 * TRACK MODEL - where the corridor is sampled
 * Row 0 is nearest the car, the last row is farthest ahead.
 * ===================================================================*/
#define TRK_ROWS                   8
#define TRK_ROW_Y_INIT             { 50, 44, 38, 32, 26, 20, 13, 6 }

/* Vectors flatter than this are start/finish lines or intersection bars, not
 * track edges. Steering off one of those ends a race. Measured in grid rows. */
#define TRK_MIN_VECTOR_DY          4

/* Vectors shorter than this are noise. */
#define TRK_MIN_VECTOR_LEN         5.0f

/* How far a vector may be extended past its own endpoints to cover a sample row.
 *
 * These are deliberately lopsided. Extending a line *toward* the car is nearly free:
 * the bottom of the frame only covers the first 20 cm or so of track, and a black
 * line does not bend much in 20 cm. It is also necessary - the camera has about a
 * 60 degree horizontal view, so close to the car the track is wider than the frame
 * and both lines leave the picture sideways even though they are plainly still there.
 *
 * Extending a line *away* from the car is the opposite: near the horizon a couple of
 * rows are a long way down the road, and guessing there is how a car ends up turning
 * into a corner that does not exist. So that direction stays tight. */
#define TRK_EXTRAP_NEAR_ROWS       10.0f
#define TRK_EXTRAP_FAR_ROWS        3.0f

/* A vector is only allowed to be extended by this multiple of its own length. A
 * long confident segment may reach a good way past its ends; a stubby one near the
 * horizon, where the line is almost vertical on screen, may barely move at all.
 * Without this, a 7 row scrap at the top of the frame gets stretched all the way
 * down to the bumper and reports a track edge that is nowhere near the real one. */
#define TRK_EXTRAP_SPAN_K          1.5f

/* The bottom rows are usually outside the camera's sideways view, so once the
 * corridor has been built from the rows that were really seen, it is continued
 * downward to fill them in. This is how many leading rows may be filled that way. */
#define TRK_FILL_DOWN_MAX          5

/* Corridor width model, in pixels. Only a seed for the first frame or two - the car
 * measures the real thing and fits its own profile (see below). */
#define TRK_WIDTH_NEAR_PX          120.0f
#define TRK_WIDTH_FAR_PX           26.0f

/* The width profile is not eight independent numbers. Flat ground and a fixed camera
 * make the corridor width a straight line in image row: twice as far away is half as
 * wide on screen. So the car fits width = a*row + b to whatever rows saw both lines
 * this frame, and that one fit calibrates every row - including the near rows, which
 * on a real car almost never see both lines at once.
 *
 * Learning rate, and the faster rate used while the car is still sitting on the grid. */
#define TRK_WIDTH_ALPHA            0.06f
#define TRK_WIDTH_ALPHA_FAST       0.35f
#define TRK_WIDTH_FAST_FRAMES      60

/* A measured width outside these multiples of the model is rejected as garbage. */
#define TRK_WIDTH_MIN_RATIO        0.45f
#define TRK_WIDTH_MAX_RATIO        1.90f

/* When only one edge is visible the corridor is assumed slightly narrower than the
 * model. That biases the car toward the edge it can actually see, which is always
 * safer than drifting toward an edge it is only guessing at. */
#define TRK_ONE_EDGE_SHRINK        0.92f

/*
 * How far ahead the car is allowed to believe what it sees - the horizon guard.
 *
 * This cannot be a number of image rows, because how far a row looks depends
 * entirely on how the camera happens to be mounted. The same row 13 is 70 cm ahead
 * on one car and 3 metres ahead on another. Aim a camera slightly flatter and the
 * top of the frame starts reporting the far side of the next corner as if it were
 * the road ahead, and the car turns in for a corner it has already been through.
 *
 * There is a mounting-independent measure though. Corridor width on screen is
 * inversely proportional to distance, so distance measured in track widths is just
 * focal length divided by the corridor width in pixels. That needs no calibration:
 * the focal length is a property of the Pixy2 grid, not of the bracket it is bolted
 * to, and the width is something the car measures for itself every frame.
 *
 * So the rule is simply "do not look more than this many track widths ahead", and it
 * holds for any camera height and any camera angle. Rows past it are dropped, the
 * car sees less far, and the speed planner slows down accordingly - which is the
 * right answer for a badly aimed camera.
 */
#define PIXY_FOCAL_PX              68.0f  /* Pixy2 line grid, ~60 degree view over 79 px */

#ifndef TRK_MAX_LOOKAHEAD_WIDTHS
#define TRK_MAX_LOOKAHEAD_WIDTHS   3.0f
#endif

#define TRK_MIN_ROW_WIDTH_PX       (PIXY_FOCAL_PX / TRK_MAX_LOOKAHEAD_WIDTHS)

/*
 * Continuity: how far the corridor centre may move between two neighbouring sample
 * rows, as a fraction of the corridor width there.
 *
 * A real road bends. It does not teleport. In a tight 180 the far rows are looking
 * most of the way round the corner, and it becomes easy for the search to lock onto
 * the far side of the same bend, or onto the outside line of the next one. When that
 * happens the corridor centre jumps by more than a whole track width in one row, and
 * the far heading flips sign from one frame to the next - which is exactly the moment
 * the car needs a steady answer.
 *
 * So the model is simply cut off at the first row that jumps. The car then sees less
 * far, slows down, and drives the part of the road it is actually sure about. */
#define TRK_MAX_CENTER_STEP_FRAC   0.90f

/* =====================================================================
 * SAFETY - the promise that the car stays inside the black lines
 * ===================================================================*/
/* Keep-out band beside each line, as a fraction of the corridor width at that row,
 * with an absolute floor in pixels. The racing line may never cross into it. */
#ifndef SAFE_MARGIN_FRAC
#define SAFE_MARGIN_FRAC           0.22f
#endif
#define SAFE_MARGIN_MIN_PX         5.0f

/* Shape of the path the car is assumed to follow toward the aim point when the
 * safety check runs. 1.0 = straight chord, 2.0 = lazy arc. 1.5 fits a car that
 * starts aligned and turns in. */
#define PATH_SHAPE_EXP             1.5f

/* =====================================================================
 * RACING LINE - outside, inside, outside
 * Bias is a fraction of the usable half-corridor. + = toward the right edge.
 * ===================================================================*/
#ifndef LINE_APEX_BIAS
#define LINE_APEX_BIAS             0.85f
#endif   /* dive to the inside at the apex     */
#define LINE_ENTRY_BIAS            0.75f   /* hold the outside on the way in     */
#define LINE_EXIT_BIAS             0.45f   /* let it run wide on the way out     */
#define LINE_BIAS_ALPHA            0.25f   /* smoothing, per new camera frame    */

/* Heading magnitude, in pixels of x per row of y, that counts as "fully in a corner".
 * Used to weigh entry / apex / exit against each other. */
#define LINE_HEAD_REF              1.10f

/* Same idea for the entry phase alone, and deliberately smaller.
 *
 * Getting to the outside has to start early - by the time the corner is big in the
 * picture the car is already in it, and there is no room left to reposition. So the
 * entry weight reaches full strength on a far heading well below a full corner,
 * while the apex weight still waits for the real thing. */
#define LINE_ENTRY_HEAD_REF        0.60f

/* Aim point. Slow = look close and be precise, fast = look far and be smooth.
 *
 * LA_ROW_MAX is 7, the farthest row, on purpose. The Pixy2 merges a smooth curve into
 * one long straight vector, and a straight line drawn across an arc touches the real
 * line only at its two ends - in between it sits on the inside of the curve. Aiming at
 * a middle row therefore aims inside the corner, and the car turns in early. The far
 * row lands on the far end of that vector, where the error goes back to zero. */
#ifndef LINE_LA_ROW_MIN
#define LINE_LA_ROW_MIN            3
#endif
#ifndef LINE_LA_ROW_MAX
#define LINE_LA_ROW_MAX            6
#endif

/*
 * How much the racing line is trusted when the camera has not given enough pieces to
 * measure curvature at all.
 *
 * A single vector per edge is a chord. It says which way the track is going on average
 * but nothing about how it bends, so headNear and headFar come out equal and the car
 * reads a hard corner as a straight road at an angle. Worse, the chord sits on the
 * inside of the bend, and the apex bias also pulls to the inside - the two errors
 * compound and the car cuts the corner.
 *
 * So when the vector count is low the bias is scaled back toward the centre line. With
 * three or more vectors the curve is properly described and the full racing line is
 * used. This is the belt; the braces are the PixyMon "Maximum merge distance" setting,
 * which is what stops the camera merging curves in the first place. */
#define LINE_CONF_SEGS_FULL        4    /* this many vectors = full confidence */
#ifndef LINE_CONF_MIN
#define LINE_CONF_MIN              0.45f

/* Extra look-ahead rows used when the curve has been chorded, to put the aim point
 * on the far end of the chord where it touches the real line again. */
#ifndef LINE_LA_LOWCONF_BOOST
#define LINE_LA_LOWCONF_BOOST      2.0f
#endif
#endif

/* =====================================================================
 * CHICANES - ignore the small ones
 * ===================================================================*/
/*
 * What counts as a corner, and what is just the road not being perfectly straight.
 *
 * Two signals are used and headNear is deliberately not one of them. Geometry: a
 * camera at height h looking at flat ground sees a lateral position error e as a
 * near-field slope of about e/h, whatever the road is doing. So headNear mostly
 * reports "the car is off to one side", not "there is a corner" - reading corners
 * off it makes the car brake for its own untidiness.
 *
 * headFar leans on the far rows, where curvature dominates, and curv is
 * headFar - headNear, which cancels the position error and leaves the bend itself.
 *
 * Below IGNORE the feature scores zero and is driven straight through at speed.
 * At FULL it gets the whole corner treatment. Measured values for reference:
 *
 *     dead straight        headFar ~0.05   curv ~0.1
 *     8 cm kink            headFar ~0.4    curv ~1.1
 *     shallow chicane      headFar ~0.9    curv ~1.4
 *     proper S bend        headFar ~2.5    curv ~3.6
 *     90 cm radius corner  headFar ~1.2    curv ~1.5
 */
#define CORNER_HEAD_IGNORE         0.45f
#define CORNER_HEAD_FULL           1.20f
#define CORNER_CURV_IGNORE         1.20f
#define CORNER_CURV_FULL           2.60f

/* Used by the explicit S-shape detector: near and far bending opposite ways, with
 * neither of them bigger than this, is a chicane to be driven straight through. */
#define CHICANE_MAX_HEAD           0.50f

/* Steering deadband, as a fraction of full lock. It slides between these two:
 * the wide one when the road ahead is barely bending, the narrow one once it is
 * clearly a corner. Soft edged, so the response never steps.
 *
 * This is what "only steer in the corners" actually means in code. It is safe to
 * ignore a kink because the safety check still owns the black lines: the moment
 * driving straight would put the car near one, the deadband is dropped to zero and
 * the car steers. So a long gentle curve is not ignored forever - the car drifts
 * across the track, using its width, and only corrects when it has to. Which is
 * precisely what a driver does. */
#define CHICANE_DEADBAND           0.08f
#define CHICANE_DEADBAND_BIG       0.34f

/* =====================================================================
 * INTERSECTIONS - recognise a crossing by the hole it leaves, and go over it
 *
 * See include/intersection.h for the method. These are the numbers.
 *
 * A crossing is not detected as a corner. It is detected as a break: the edge the
 * car is following stops, there is about one track width of white space, and then
 * the same edge picks up again, parallel to where it left off and in line with it.
 * The bars across the mouth of the crossing are not used - they are thrown away
 * with everything else that does not run up the track.
 *
 * Everything below that describes a place on the track is in CENTIMETRES on the
 * ground, because the scan unprojects the frame before it measures anything. The
 * same gap is thirty image rows deep at the bumper and three near the horizon, so
 * there is no pixel threshold that means one thing at both ends of the picture.
 *
 * Tuning order that works:
 *   1. CAM_HORIZON_ROW / CAM_HEIGHT_CM above - get these right first. Every
 *      distance here is measured through them, and if they are wrong nothing else
 *      in this block will help.
 *   2. ISEC_GAP_MIN_CM / ISEC_GAP_MAX_CM - the width of the crossing track, with
 *      room either side. This is the main thing that says yes or no.
 *   3. ISEC_COMMIT_CM - how late the car commits.
 *   4. ISEC_HEAD_GAIN - how hard it lines itself up with the track.
 * ===================================================================*/
/* Set to 0 and the whole feature compiles out: no detection, no override, and the
 * car behaves exactly as it did before the module existed. */
#define ISEC_ENABLE                1

/* An endpoint this close to the horizon is at an unusable distance - a single row
 * of quantisation moves it by a large fraction of how far away it is - so a vector
 * with an end up there is dropped rather than unprojected. */
#define ISEC_MIN_ROWS_BELOW_HZ     6.0f

/* What counts as a line running UP the track rather than across it: it must go at
 * least this much further away than it goes sideways. This is the test that throws
 * the crossing bars away, so it does not need to be tight - 1.0 is 45 degrees, and
 * a bar is nearer 90. */
#define ISEC_LONG_RATIO            1.00f

/* Shortest edge worth believing, on the ground. The camera splits one black line
 * into a chain of short vectors and they get shorter the closer the car gets, so
 * this has to stay small - it is only here to keep single-pixel noise out. What
 * stops a short vector's wobbly direction mattering is not this number: the scan
 * merges the chain back into one edge before measuring anything, and the parallel
 * test picks the longest piece on each side of the hole rather than the nearest. */
#define ISEC_MIN_EDGE_CM           5.0f

/* How far to one side of the car a line may be and still be part of its own track:
 * half a track width plus however far off centre the car is. Further out than this
 * belongs to another part of the circuit and must not be followed. */
#define ISEC_SIDE_MAX_CM           55.0f

/* The forward scan: how finely it looks and how many bins deep, so ISEC_BINS times
 * ISEC_BIN_CM is how far ahead it looks. The bin size is what makes overlapping,
 * abutting and slightly separated vectors all read as one continuous edge, so it
 * wants to be comfortably bigger than the gap the camera leaves between two pieces
 * of the same line and comfortably smaller than the crossing. ISEC_BINS is written
 * as a plain integer because it is an array size in intersection.c; keep it 127 or
 * less and the scan off the stack. */
#define ISEC_BIN_CM                5.0f
#define ISEC_BINS                  40u
#define ISEC_SCAN_CM               ((float)ISEC_BINS * ISEC_BIN_CM)

/* The near piece has to reach at least this close to the car. An edge that only
 * appears out in the distance is not one the car is following, and the hole beyond
 * it says nothing about the track under the wheels. */
#define ISEC_EDGE_START_MAX_CM     70.0f

/* The white space. A crossing track is as wide as this one, so the hole it leaves
 * is about one track width - these are that, with room either side for the camera
 * losing a little of each edge at the mouth. Tighten them if something else on the
 * circuit is being read as a crossing. */
#define ISEC_GAP_MIN_CM            25.0f
#define ISEC_GAP_MAX_CM            95.0f

/*
 * Require the edge to pick up again on the far side of the hole.
 *
 * This is what separates a crossing from an edge that has simply run out - the
 * camera reaching the end of its look-ahead, or the inside line leaving the side
 * of the frame in a corner, both of which happen constantly. Leave it on. Turning
 * it off makes the detector fire on any edge that stops, which on a real circuit
 * is most of them.
 */
#define ISEC_REQUIRE_FAR_EDGE      1

/* How much of the far side of the hole has to be black line. A crossing puts a
 * whole track's worth of edge over there; a stub that happens to land beyond a gap
 * is what gets through when the camera calibration is out. */
#define ISEC_MIN_FAR_CM            15.0f

/* How alike the two pieces have to be to count as the same black line: parallel to
 * within this much sideways per forward, and in line to within this many
 * centimetres where the far piece starts. A real crossing is in line to within the
 * grid quantisation, so these have plenty of room in them. */
#define ISEC_PARALLEL_TOL          0.30f
#define ISEC_COLLINEAR_CM          15.0f

/*
 * The other way a crossing shows itself: the car is already on its doorstep.
 *
 * From a distance the far side of the hole is in frame and the test above finds
 * it. Close up it is not - the far edges are a few pixels tall at the very top of
 * the picture and the camera often does not report them at all. What is left is
 * both black lines stopping dead a short way ahead with nothing beyond either of
 * them, which is a crossing seen from a metre away and is also exactly the frame
 * in which the car most needs to decide to go straight.
 *
 * BOTH lines have to stop, and within ISEC_MOUTH_SKEW_CM of each other. That is
 * what keeps a corner out: in a corner the inside line leaves the side of the
 * frame long before the outside one runs out of look-ahead, so the two ends are
 * nowhere near each other, while a crossing cuts both with the same straight edge.
 *
 * Since the far side cannot be measured from here, it is assumed to be
 * ISEC_BLIND_CROSS_CM away - one track width, which is what a crossing is.
 *
 * IT SHIPS OFF, AND IT IS GATED ON THE CAMERA CONSTANTS BEING RIGHT.
 *
 * With CAM_HEIGHT_CM and CAM_HORIZON_ROW matching the real mounting it is free:
 * every circuit, chicane and camera-failure test in test/track_sim.c runs
 * identically with it on and off, and it picks up crossings the far-side test
 * cannot see. With those constants wrong it is dangerous, because "both lines stop
 * and there is white space beyond" is then also what a badly aimed camera reports
 * in an ordinary corner - and the car drives straight out of it. Measured, across
 * the 162 mountings of the robustness sweep: a dozen runs that finished no longer
 * do, one of them inside the recommended mounting envelope.
 *
 * So: measure the two camera numbers, check them, and then set this to 1. Not
 * before. Everything below it is live either way and needs no retuning.
 */
#define ISEC_MOUTH_ENABLE          0
#define ISEC_MOUTH_CM              55.0f
#define ISEC_MOUTH_SKEW_CM         25.0f
#define ISEC_BLIND_CROSS_CM        55.0f

/*
 * Require a black line lying ACROSS the track, at or beyond where the edges
 * stopped, before the doorstep test is believed.
 *
 * This is the one place the crossing bars are used, and it is not for steering -
 * they are still never followed. It is for evidence. "Both lines stop and there is
 * white space beyond" is on its own a very weak signature, because it is also what
 * a camera that simply cannot see very far reports, all the time. Across the 162
 * mountings in the robustness sweep, leaving this off turns a dozen runs that
 * finished into runs that drive straight out of a corner. With it on the sweep is
 * untouched. Leave it on.
 */
#define ISEC_MOUTH_NEEDS_BAR       1

/* Shortest sideways run that counts as a line lying across the track, and how far
 * either side of the expected crossing the bar may sit and still be believed. */
#define ISEC_MIN_BAR_CM            15.0f
#define ISEC_BAR_SLACK_CM          25.0f

/* How much of the line has to have been in view before it stops. A two centimetre
 * fleck that ends is not a line the car was following. */
#define ISEC_MOUTH_MIN_RUN_CM      10.0f

/*
 * Accept one line stopping when the other is not in the picture at all.
 *
 * A car arriving off centre cannot see its far edge - a 60 degree view does not
 * reach it until it is most of a metre away - so close to a crossing there really
 * is only one line to go on, and insisting on two means never recognising the
 * crossings the car is worst placed for. This is NOT the same as one line stopping
 * while the other carries on, which is a corner and is refused whatever this is
 * set to: in a corner the outside line is the one the camera sees best, and it
 * does not stop. Set to 0 to require both lines every time.
 *
 * With only one line there is no second opinion, so the line itself has to look
 * like a crossing approach - running very nearly parallel to the car, to within
 * ISEC_MOUTH_STRAIGHT sideways per forward. In a hairpin the one line the camera
 * can hold onto sweeps away across the frame, and that is what this refuses.
 *
 * It ships OFF, because that is not enough. With it on, all four test circuits
 * stay clean - but across the 162 camera mountings in the robustness sweep it
 * turns a dozen runs that finished into runs that leave the track, including one
 * inside the recommended mounting envelope. A hairpin where the outside line is
 * momentarily out of frame reads as a crossing, and the car drives straight on.
 *
 * So the cost of leaving it off is real and known: a crossing arrived at well off
 * centre, close enough that the far edges are not in frame, is not recognised and
 * is driven as ordinary track. The cost of turning it on is a car that
 * occasionally drives straight out of a corner. Only turn it on with a flight
 * recording that shows crossings actually being missed this way.
 */
#define ISEC_MOUTH_ONE_SIDED       0
#define ISEC_MOUTH_STRAIGHT        0.20f

/* Frames of agreement before the module is allowed to commit. Counts up on a
 * detection and down on a miss, so one dropped frame does not undo it. */
#define ISEC_CONFIRM_FRAMES        2

/* The car commits when the mouth of the crossing is this close. Until then the
 * module is exactly invisible - it does not touch the steering at all, so on a
 * circuit with no crossings on it the car drives as if this file did not exist. */
#define ISEC_COMMIT_CM             60.0f

/* How far past the far side of the hole to keep driving straight, so the back of
 * the car is out of the crossing before the corridor is believed again. */
#define ISEC_CLEAR_CM              35.0f

/* The latch distance is measured, not guessed: it is however far the far side of
 * the hole was, plus ISEC_CLEAR_CM. These only stop a silly measurement buying a
 * silly amount of blind driving. */
#define ISEC_CROSS_MIN_M           0.60f
#define ISEC_CROSS_MAX_M           1.60f

/* After a crossing, detections are ignored for this far. Coming out of one, the
 * edges behind look exactly like the near side of another. */
#define ISEC_COOLDOWN_M            0.60f

/* Backstop for a car that is not moving: no ground covered means the distance
 * budgets above never expire. Neither phase may outlast this. */
#define ISEC_PHASE_MAX_MS          3000.0f

/*
 * Lining up with the track.
 *
 * The visible edges have a heading relative to the car, measured on the ground,
 * and this is the gain that nulls it - a plain P controller on "am I parallel".
 * Holding it means the car leaves the crossing on the line it entered on, which is
 * what driving straight over one means.
 *
 * A slope of 1.0 is 45 degrees off, which nothing survives, so with a gain of 30
 * and a clamp of 25 the command saturates at about 40 degrees off and is gentle
 * anywhere near straight.
 */
#define ISEC_HEAD_GAIN             30.0f
#define ISEC_STEER_MAX             25.0f

/* If a frame comes back with nothing running up the track at all, the last command
 * is faded toward straight ahead by this factor rather than being held. */
#define ISEC_ALIGN_DECAY           0.75f

/* Speed ceiling while crossing. The car is driving on a latch rather than on what
 * it can see, so it should not be doing it flat out. Raise to SPEED_MAX to remove
 * the cap entirely. */
#define ISEC_SPEED_CAP             85.0f

/* =====================================================================
 * STEERING
 * Output units match Steer(): -100 .. +100, positive = right.
 * ===================================================================*/
#define STEER_OFFSET               (-13.0f) /* servo value that points the wheels straight */
#define STEER_LIMIT_RIGHT          45.0f
#define STEER_LIMIT_LEFT           (-60.0f)

/* Left and right trim, on top of the limits above.
 *
 * Leave these at 1.0. The car already steers less freely to the right and that is
 * expressed once, in STEER_LIMIT_RIGHT. Putting the same asymmetry in a second time
 * as a gain means the controller cannot reach the lock it does have - which shows up
 * as the car quietly running wide out of every right hand corner.
 *
 * Only touch these if the car is measurably lazier on one side at the same command. */
#define STEER_GAIN_RIGHT           1.00f
#define STEER_GAIN_LEFT            1.00f

/* Controller. KP acts on how far the aim point sits beside the car, KH on which way
 * the track points right in front of the bumper, KD damps the pair. */
#define STEER_KP                   78.0f
#define STEER_KH                   30.0f
#define STEER_KD                   4.0f

/* =====================================================================
 * GEOMETRIC STEERING (pure pursuit)
 *
 * STEER_KP is one number for every situation, but the thing it multiplies is
 * not. The aim point is measured in pixels, and a pixel is worth more sideways
 * travel the farther away the row is - so as LINE_LA_ROW slides outward with
 * speed, the same KP means a different steering gain. sim/steering.py works out
 * what the geometrically correct gain is at each look-ahead distance, and the
 * fixed 78 crosses it at about 0.48 m: the car is under-geared below that and
 * over-geared above it, which is why it runs wide out of slow corners and cuts
 * into fast ones.
 *
 * The geometrically right law is pure pursuit. To reach a point y_la to the side
 * at distance d_la, a bicycle of wheelbase L needs curvature 2*y_la/d_la^2, so
 *
 *     delta = 2 * L * y_la / d_la^2
 *
 * Both y_la and d_la come from the same camera, and the focal length cancels
 * almost entirely. With Dx the aim point's pixel offset and w the corridor width
 * in pixels at that row (which is how the firmware already measures distance,
 * because width is inversely proportional to it):
 *
 *     y_la = Dx * d_la / f        d_la = f * W / w
 *
 *     delta = 2 * L * Dx * w / (f^2 * W)
 *
 * So the command is simply the aim-point offset multiplied by the corridor width
 * there, and the only calibration left is L/W - wheelbase over track width, a
 * ratio you can measure with a tape. No camera height, no tilt, no focal length
 * beyond the Pixy2's own grid constant. Aim the camera differently and this law
 * does not need retuning.
 *
 * It also removes the need for a separate curvature feedforward. On a curved
 * path the aim point is already displaced by the arc's own sagitta, and pure
 * pursuit converts exactly that displacement into exactly the steering angle the
 * corner needs. Adding an explicit feedforward on top double-counts it, turns the
 * car in early and cuts the apex - which is what STEER_KFF was measured doing.
 * ===================================================================*/
#ifndef STEER_PURE_PURSUIT
#define STEER_PURE_PURSUIT         1
#endif

/* Car geometry. Only the ratio of these two matters. */
#define CAR_WHEELBASE_M            0.170f
#define TRACK_WIDTH_M              0.450f

/* Road-wheel angle at a steering command of 100, radians. 30 degrees. */
#define CAR_MAX_STEER_RAD          0.5236f

#define STEER_PP_K                 ((200.0f * (CAR_WHEELBASE_M / TRACK_WIDTH_M)) \
                                    / (PIXY_FOCAL_PX * PIXY_FOCAL_PX * CAR_MAX_STEER_RAD))

/* Multiplier on the geometric value. 1.0 is textbook pure pursuit, which tracks
 * a corner exactly but is leisurely about correcting a disturbance. Above 1.0
 * corrects harder and tucks the car inside the geometric line. */
#ifndef STEER_PP_SCALE
#define STEER_PP_SCALE             1.30f
#endif

/* Heading that saturates the KH term, in pixels of x per row of y. */
#define STEER_HEAD_SCALE           1.10f

/*
 * Curvature feedforward, steering units per unit of track curv.
 *
 * A constant-radius corner needs a constant steering angle, delta = L/R. A
 * proportional controller can only produce a constant output from a standing
 * error, so with KP alone the car MUST sit off the line by (L/R)/K for as long
 * as the corner lasts. That is not a tuning fault, it is what proportional
 * control is - and on a 90 cm corner it works out at about a quarter of a metre,
 * which is more than the half-width of the track. It is exactly the error that
 * stops a car holding an apex.
 *
 * track.c already measures curv = headFar - headNear, and sim/steering.py shows
 * curv is proportional to real curvature at 0.97 (px/row) per (1/m) across the
 * whole useful radius range. So the angle the corner needs can simply be handed
 * to the servo before the error term is asked for any of it:
 *
 *     steer_ff = 100 * L * (curv / 0.97) / MAX_STEER_RAD  =  33 * curv
 *
 * Feedforward sits outside the loop, so it changes no margin and cannot
 * destabilise anything. The default is deliberately below the geometric 33:
 * the estimate runs short in tight corners, and under-delivering is the safe
 * direction because the proportional term simply picks up the remainder.
 */
#ifndef STEER_KFF
#define STEER_KFF                  0.0f
#endif

/* Largest curvature the feedforward will believe, in the same px-per-row units.
 * 2.0 corresponds to a half-metre radius - tighter than an NXP Cup layout puts
 * down, so anything past it is an S bend being misread as a corner rather than a
 * corner that is really that tight. */
#ifndef STEER_KFF_CURV_MAX
#define STEER_KFF_CURV_MAX         2.00f
#endif

/* Servo rate limit, steering units per second. Stops the linkage from slamming. */
#define STEER_SLEW_PER_S           900.0f

/* The D term runs on this much smoothing, to keep camera noise out of the servo. */
#define STEER_D_ALPHA              0.40f

/* =====================================================================
 * SPEED - as fast as the situation allows, every single frame
 * ===================================================================*/
/* THE headline number. Start at 65, raise it 5 at a time. */
#ifndef SPEED_MAX
#define SPEED_MAX                  100.0f
#endif

/*
 * Slowest the car is allowed to go while it can still see the track.
 *
 * This, SPEED_SEVERITY_FULL and STEER_PP_SCALE are the three that trade lap time
 * against margin. test/pick.py scores a candidate on all three test sets at once
 * - the five circuits, the fault suite, and the 18 camera mountings inside the
 * aim envelope - because a tuning search given only lap time will happily sell
 * the mounting robustness to buy a second, and the car then depends on the
 * bracket being exactly where it was when the tune was found.
 *
 * Measured ladder (total of the five circuits, all with zero line contact):
 *
 *   MIN 60, SEV 1.00, PP 1.6   48.69 s   18/18 mountings  8/8 faults
 *   MIN 68, SEV 1.00, PP 1.6   44.78 s   16/18 mountings  8/8 faults
 *   MIN 76, SEV 1.30, PP 1.3   41.74 s   15/18 mountings  8/8 faults  <- default
 *
 * For reference the old open-loop firmware managed 51.76 s at 18/18 and 7/8, and
 * 43.08 s at 16/18 and 5/8.
 *
 * What the default gives up, precisely. `track_sim -envelope` lists all 18 and
 * marks each one. The three the default fails are:
 *
 *   f=55 h=26 hz=-6  track=45 R=60      f=55 h=26 hz=-6  track=45 R=80
 *   f=80 h=26 hz=-10 track=45 R=80
 *
 * Every single one is the narrowest track, 45 cm. On a 55 cm or 60 cm track the
 * default is clean on all ten of those mountings, and at the Pixy2's real focal
 * length (68 px; the sweep varies it 55-80 to cover uncertainty in that constant)
 * it is clean on all four with 3.2 to 8.5 cm to spare.
 *
 * So: TRACK WIDTH is the thing to check, not the bracket. Measured on a 45 cm
 * track, counting how many of those eight mountings cross a line:
 *
 *   MIN 76, SEV 1.30, PP 1.3   41.74 s   3 of 8 fail   worst -34.5 cm
 *   MIN 68, SEV 1.00, PP 1.6   44.78 s   2 of 8 fail   worst  -9.0 cm
 *   MIN 64, SEV 1.00, PP 1.6   44.97 s   1 of 8 fail   worst  -6.5 cm
 *   MIN 60, SEV 1.00, PP 1.6   48.69 s   0 of 8 fail   worst  +0.3 cm
 *
 * On a 45 cm track only the last one is safe, and even that clears by 3 mm.
 */
#ifndef SPEED_MIN
#define SPEED_MIN                  76.0f
#endif

/* Global scale, handy for a quick trackside calm-down. 1.0 = full. */
#define SPEED_SCALE                1.00f

/* How strongly each cue slows the car. The largest wins, so a corner seen far ahead
 * brakes the car early even while the road right in front is still straight. */
#define SPEED_W_HEAD_FAR           1.05f
#define SPEED_W_CURV               0.85f
#define SPEED_W_STEER              0.90f

/* Corner severity that pins the car at SPEED_MIN. */
#ifndef SPEED_SEVERITY_FULL
#define SPEED_SEVERITY_FULL        1.30f
#endif

/* You may only drive as fast as you can see. Speed is scaled by how far ahead the
 * track model is still valid. */
#define SPEED_SEE_ROWS_FULL        6       /* this many valid rows = no penalty */
#define SPEED_SEE_ROWS_MIN         2       /* at or below this = crawl          */
#ifndef SPEED_SEE_FLOOR
#define SPEED_SEE_FLOOR            0.45f
#endif

/* Cap while only one edge of the track is visible. */
#ifndef SPEED_ONE_EDGE_CAP
#define SPEED_ONE_EDGE_CAP         0.92f
#endif

/* Acceleration is ramped, braking is instant - same as a real car. Units per second. */
#define SPEED_ACCEL_PER_S          140.0f
#define SPEED_ACCEL_EXIT_PER_S     260.0f  /* corner exit: get on the power hard */

/* =====================================================================
 * LONGITUDINAL PLANT - what the motors actually do
 *
 * Identified in sim/plant.py from the 25GA-370 datasheet points (12 V, 1000 rpm
 * out, 0.12 A no load, 3.0 A stall -> 1:9.6 gearing, R = 4.36 ohm, Ke = Kt =
 * 0.01146, tau_mech = 348 ms) and validated there against a PWM-resolved model
 * that integrates the winding current at 2 us. Re-run sim/identify.py after
 * changing any of it.
 *
 *     duty -> wheel speed   G(s) = K / (tau*s + 1)
 *
 * If the car is measurably faster or slower than SPEED_TOP_MS on a straight,
 * that is the number to correct first: everything else is scaled by it.
 * ===================================================================*/

/* Steady speed at 100% duty, m/s. Scales with battery voltage - 2.01 at 8.0 V,
 * 1.73 at 7.0 V, 3.12 at a full 12 V. Measure it once with a tape and a stopwatch. */
#ifndef SPEED_TOP_MS
#define SPEED_TOP_MS               2.01f
#endif

#define MOTOR_K_MS_PER_DUTY        2.226f  /* m/s per unit duty, no load */
#define MOTOR_TAU_S                0.348f  /* dominant pole */

/*
 * Which way the bridge decays during the PWM off-time, going forwards.
 *
 * The wiring is one GPIO direction pin plus one PWM pin per motor, and that
 * gives coast-during-off in one direction and brake-during-off in the other.
 * Forwards is currently the coast one, which is the worse half of the deal:
 * a 30% deadband, an S-shaped duty-speed curve, twice the speed lost to a given
 * drag, and three times the current ripple. Brake decay is nearly a straight
 * line through the origin.
 *
 * Swapping the two motor wires at both motors and setting MOTOR1_INVERT and
 * MOTOR2_INVERT to 1 puts forward on the brake-decay branch instead, at which
 * point set this to 0. Worth about a second a lap - test/compare.sh measures it.
 */
#ifndef MOTOR_FAST_DECAY_FWD
#define MOTOR_FAST_DECAY_FWD       1
#endif

/* Set to 1 once a wheel speed sensor exists and SpeedCtl_Measure() is being
 * called. Only the integral term needs it, but the integral term is the only
 * thing that can reject a drag disturbance. */
#ifndef SPEED_HAVE_FEEDBACK
#define SPEED_HAVE_FEEDBACK        0
#endif

/* PI gains, by pole cancellation at a 12 rad/s closed loop: Kp = tau*w/K,
 * Ki = Kp/tau. Expressed in duty percent per (m/s). Unused unless
 * SPEED_HAVE_FEEDBACK is 1. */
#define SPEED_KP                   187.6f
#define SPEED_KI                   539.1f
#define SPEED_I_LIMIT              0.60f

/* How much of the 348 ms lag to invert. 1.0 is exact inversion; below 1.0 is
 * gentler on the bridge, above 1.0 overdrives. Drop it if the car surges. */
#ifndef SPEED_FF_LAG
#define SPEED_FF_LAG               1.00f
#endif

/* Reference shaping, m/s^2. The car can manage about 3.5 m/s^2 at mid speed,
 * so asking for much more than that only saturates the bridge. The decel limit
 * is what the feedforward turns into a reverse command; COAST is used instead
 * whenever braking is not allowed. */
#define SPEED_ACC_MS2              3.00f
#define SPEED_ACC_EXIT_MS2         6.00f  /* corner exit: let the reference run
                                           * ahead so the duty pins at 100% */
#define SPEED_DEC_MS2              6.00f
#define SPEED_COAST_MS2            2.50f

/* Master switch. 0 falls back to the original open-loop ramp and reverse brake
 * pulse, which is how test/compare.sh measures what this is worth. */
#ifndef SPEED_CLOSED_LOOP
#define SPEED_CLOSED_LOOP          1
#endif

/* =====================================================================
 * BRAKING - short reverse pulse when the speed demand drops hard
 * ===================================================================*/
#define BRAKE_ENABLE               1
#define BRAKE_TRIGGER              16.0f   /* speed-unit deficit that starts a pulse */
#define BRAKE_REVERSE_MAX          28.0f   /* strongest reverse allowed              */
#define BRAKE_MAX_MS               90.0f   /* never brake longer than this           */

/* =====================================================================
 * TORQUE VECTORING - slow the inside wheel through a corner
 * ===================================================================*/
#ifndef DIFF_GAIN
#define DIFF_GAIN                  0.55f
#endif   /* 0 = off, 0.45 = inner wheel at 55% at full lock */

/* =====================================================================
 * MOTORS
 * ===================================================================*/
/* Set to 1 if a wheel spins backwards. Motor 1 is the left side. */
#define MOTOR1_INVERT              0
#define MOTOR2_INVERT              0

/* Set to 1 if the left and right motor connectors are swapped. */
#define MOTOR_SWAP_SIDES           0

/* =====================================================================
 * START-UP AND FAILSAFE
 * ===================================================================*/
/* Wheels stay dead this long after power-up so the car can be placed safely. */
#define START_DELAY_MS             1500.0f

/* Speed ramp from standstill. */
#define START_RAMP_MS              600.0f

/* Losing the track: coast, then slow, then stop. Counted in camera frames.
 *
 * A frame or two with no line is normal and is coasted through. After that the car
 * sheds speed quickly, because everything it does while blind is guesswork and the
 * only thing that limits how wrong that guess can get is how far it travels. */
#define LOST_COAST_FRAMES          3
#define LOST_SLOW_FRAMES           10
#define LOST_STOP_FRAMES           30
#define SPEED_LOST                 20.0f

/* The same three stages, measured in metres of track covered blind rather than
 * in camera frames. Distance is the honest unit: a frame is a different amount
 * of ground on a straight than in a hairpin, and a different amount again if the
 * camera rate drops. The frame counters above only behaved because the motor lag
 * stopped the car ever reaching the speeds they asked for; a car that obeys them
 * crawls, and thirty frames of crawling is barely a hand's width of track. */
#ifndef LOST_COAST_M
#define LOST_COAST_M               0.12f
#endif
#ifndef LOST_SLOW_M
#define LOST_SLOW_M                0.35f
#endif
#ifndef LOST_STOP_M
#define LOST_STOP_M                0.75f
#endif

/* If the camera says nothing at all for this long, cut the motors. */
#define CAM_TIMEOUT_MS             350.0f

/* Slowest the car may be scaled to when the camera is delivering frames slower than
 * it normally does. "Only as fast as you can see" applies to time as well as
 * distance: at half the frame rate, every steering correction arrives twice as late,
 * so the car has to be correspondingly slower for the same margin of safety.
 *
 * The healthy frame rate is learned, not assumed, so a 30 Hz camera is not punished
 * for being a 30 Hz camera - only for suddenly becoming a 12 Hz one. */
#define CAM_RATE_FLOOR             0.35f

#endif /* RACE_CONFIG_H */
