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
 * INTERSECTIONS - recognising a crossing and taking it flat
 *
 * Where a second piece of track crosses ours, both tracks are painted on the
 * same floor and the paint is interrupted in the overlap. So the camera sees a
 * very specific thing, and nothing else on an NXP Cup layout looks like it:
 *
 *      our two lines stop, leave a gap the width of the crossing track,
 *      and resume in line with themselves - and lying across that gap is a
 *      pair of nearly sideways lines, one either side of our corridor,
 *      which are the crossing track's own edges cut in half by ours.
 *
 * Those sideways lines are already found and thrown away by track.c, which is
 * right - steering off one would end a race. This module picks them back out of
 * the same vector list and asks whether they are arranged like a crossing.
 *
 * The point of recognising it is that the car must NOT do what it would
 * otherwise do. Through a crossing its own lines are missing, so the corridor
 * shrinks, the see-distance cue reads it as "cannot see far" and the car lifts;
 * on a wide crossing it loses the track outright and stops. Recognised, it
 * holds the wheel where it was and keeps the power on.
 * ===================================================================*/

/* ---- the splay test -------------------------------------------------------
 *
 * How far the two vectors have to be bending apart before it counts.
 *
 * This is the test that does the work on real hardware, and it is strong
 * because it tests something that cannot happen. Two parallel track edges seen
 * in perspective ALWAYS converge - whatever the camera height, whatever the
 * tilt, whatever the lens, each one leans toward the middle as it goes away.
 * Nothing about a straight, a bend or a chicane makes both lean outward at
 * once. So when they do, what the camera is following is not our two lines any
 * more: our line has run into the crossing track's line and the tracker has
 * gone round the corner and back out along it.
 *
 * Measured on each vector separately, as columns moved sideways per row of
 * going farther away, and BOTH have to be moving outward at least this much.
 * On a straight both are firmly negative by this measure; in a corner both
 * swing the same way, not apart. 0.18 is about one column in six rows, which
 * is comfortably above the quantisation of the 79 x 52 grid. */
#define XSEC_DIV_MIN_SLOPE         0.18f

/* A vector shorter than this in rows has no reliable direction - the slope of a
 * three-row scrap is mostly rounding. Measured in grid rows. */
#define XSEC_DIV_MIN_DY            6.0f

/* ---- the bar test ---------------------------------------------------------
 *
 * The other way the same crossing can arrive. Whether the camera gives you
 * splaying vectors or loose bars depends on whether it breaks the paint at the
 * corner where our line meets the crossing, and both happen - so the car
 * accepts either.
 *
 * A vector this flat is a lateral bar: a crossing edge, or a start/finish line.
 * Must stay >= TRK_MIN_VECTOR_DY or track.c would be steering off the ones this
 * module is looking at. Measured in grid rows. */
#define XSEC_BAR_MAX_DY            3.0f

/* ...and it has to be long enough sideways to be one. Grid columns. */
#define XSEC_BAR_MIN_DX            6.0f

/* Two bars within this many rows of each other are the same bar seen in pieces. */
#define XSEC_BAR_ROW_TOL           4.0f

/* A bar farther down the frame than this is under the bumper already - too late
 * to be the thing we are approaching, and usually the crossing we just left. */
#define XSEC_BAR_NEAR_Y            46.0f

/* How much of a corridor width the bars must add up to before they count. The
 * real thing comfortably exceeds one width; noise does not get close. */
#define XSEC_MIN_COVER             0.90f

/* ...spread over at least this many separate vectors. A crossing presents four
 * stubs and rarely fewer than two; a single flattened corner edge presents one. */
#define XSEC_MIN_BARS              2

/* A bar has to reach this far past the corridor centre, as a fraction of the
 * corridor width, to count as being on that side of it. */
#define XSEC_SIDE_FRAC             0.10f

/* The hole our own track cuts in the crossing's lines, as a fraction of the
 * corridor width - the single most useful test in here.
 *
 * Two tracks painted on one floor interrupt each other, so a crossing always
 * shows its edges as stubs with a track-width hole between them. The thing that
 * most resembles a crossing and is not one - the two track edges of an ordinary
 * corner flattening out as they converge on the vanishing point - reaches
 * INWARD and leaves no hole. Requiring the hole took the false alarms on clean
 * circuits from 28 a lap to none, without costing a single real detection. */
#define XSEC_GAP_FRAC              0.55f

/* ---- the elbow test -------------------------------------------------------
 *
 * The third way the same crossing arrives, and on a real Pixy2 the most common
 * one of all: our own line runs up the frame, stops dead, and the crossing
 * track's edge sets off sideways from the very point it stopped. Two vectors
 * sharing an endpoint, meeting at about a right angle - a corner - and there is
 * one on each side of the corridor:
 *
 *      ----------o                 o----------      the crossing's near edge,
 *                |                 |                broken by ours
 *                |                 |
 *                |                 |                our two lines, ending at
 *                                                   the corner they run into
 *
 * Neither of the other two tests sees this. The splay test does not, because
 * our lines still converge perfectly normally - the tracker broke the paint at
 * the corner instead of following it round, so nothing bends outward. The bar
 * test does not, because a crossing edge near the top of the frame is drawn
 * across a few rows of a 52 row grid rather than exactly one, and at
 * XSEC_BAR_MAX_DY = 3 the steeper of the two arms is thrown out as not flat
 * enough - leaving one bar where XSEC_MIN_BARS wants two.
 *
 * Loosening the bar test until it caught this would be the wrong fix: flatness
 * is all a loose bar has to recommend it, and a corner's outer edge flattens
 * out near the vanishing point too. The elbow gets its confidence from
 * somewhere else entirely, and from something that cannot happen:
 *
 *      THE TWO ARMS POINT AWAY FROM EACH OTHER.
 *
 * A corner in our own track bends one way. Both its edges turn the same way,
 * because they are two sides of one road, so the left elbow's arm and the right
 * elbow's arm both sweep left, or both sweep right. Only a second track cutting
 * across ours puts one arm out to the left and the other out to the right at the
 * same moment - that arrangement is the crossing's single edge with our corridor
 * punched through the middle of it. Same family of test as the splay: not "this
 * looks like a junction" but "no single road can look like this".
 */

/* How close two endpoints have to be, in grid cells, to count as the same
 * corner. Generous: the tracker rarely lands both vectors on the same cell, and
 * on a 79 x 52 grid three cells is still a small fraction of a track width. */
#define XSEC_ELB_JOIN              3.5f

/* Rows a vector must run to be a stem - one of our own lines going away from
 * the car, rather than a scrap. Same reasoning as XSEC_DIV_MIN_DY: the
 * direction of a three row fragment is mostly rounding. */
#define XSEC_ELB_STEM_DY           6.0f

/* Columns the arm must reach out sideways from the corner, and the direction is
 * what matters - left elbow to the left, right elbow to the right. A crossing
 * edge runs to the edge of the frame; noise hanging off the end of a line does
 * not go anywhere. */
#define XSEC_ELB_ARM_DX            4.0f

/* How far from square the corner may be, as |cos| of the angle between the stem
 * and the arm. 0.70 accepts 46 to 134 degrees.
 *
 * Wide, and deliberately so - this is a plausibility check, not the test that
 * does the work. Perspective does not preserve angles: a square crossing at the
 * top of the frame measured 114 and 104 degrees on the two sides of the frame
 * this was built from, 121 on another, and at a mounting whose horizon sits
 * just above the frame our own lines lean over a column per row, which puts a
 * level crossing edge at 138 degrees to them. Tightening it around 90 would
 * throw away real junctions to catch nothing, because what is actually being
 * kept out is a line continuing straight on through a paint gap, and that
 * arrives at 0 degrees with a mile of margin. Everything that looks like a
 * corner and is not one is caught by the tests below, none of which uses the
 * angle. Wider still - 0.80 - let the inside edge of a bend through in the
 * mounting sweep at 137 degrees, so this is as wide as it goes without another
 * test to lean on. */
#define XSEC_ELB_COS_TOL           0.70f

/* Grid columns the two corners must be apart before they are believed to be
 * opposite sides of one hole rather than two readings of one scrap of paint. */
#define XSEC_ELB_MIN_SEP           6.0f

/* Rows apart the two corners may sit. They are the two ends of one edge lying
 * across our path, so they arrive at nearly the same distance; what separates
 * them is the car being off-centre or the crossing being skewed. Generous
 * enough for both, and near the horizon - where these are first seen - rows
 * compress, so the real thing lands well inside it. */
#define XSEC_ELB_ROW_TOL           10.0f

/* Accept a crossing from ONE elbow when the rest of the frame does not
 * contradict it.
 *
 * On the real track this is the common case. Of three PixyMon frames of the
 * same junction, one showed both corners, one showed exactly three vectors -
 * our left line running up to a sharp corner, the crossing edge leaving that
 * corner to the left, and a plain unbroken line on the right - and one showed
 * no corner at all. And on that track a two-vector elbow at about a right
 * angle is something ONLY a junction produces: the curves are arcs, and the
 * camera draws an arc as a chain of shallow bends, never as one sharp corner.
 * The pair rule alone read the three-vector frame as nothing, so the car
 * lifted, then lost the lines, then steered after the crossing's own edge.
 *
 * Why it was off before, and what changed. Accepting a lone elbow was first
 * tried with only the sharp-sweep test and the opposite-stem test for company,
 * and it put the car off the track in the simulator: "tight 180s" went from
 * 0 excursions to 9037. Every one of those 232 firings was the same shape,
 * and it was the simulator's, not the track's: the simulator draws a curved
 * edge as three equal chords, so the inside edge of a bend seen from the
 * straight before it arrives as a straight stem with a level chord running
 * across the top of the frame from its end - a right angle, pointing outward,
 * exactly like a crossing edge. A camera does not draw an arc that way, but
 * something might, so rather than trust that, the frame is made to prove it
 * is not a bend. A lone elbow is believed only when:
 *
 *   - the arm lies nearly level (XSEC_ELB_ARM_FLAT). A crossing edge lies
 *     across the road, so it stays close to the horizon at any pitch and is
 *     tilted only a few degrees by a yaw; a chord of our own edge climbs.
 *   - the OTHER edge is described, does not bend sharply toward the arm's
 *     side, and does not stray toward it gradually either: its far pieces sit
 *     within XSEC_ELB_SWEEP_COLS of the straight line through its own stem.
 *     A bend turns the road as a whole; the inside edge turning across the
 *     frame is given away by the outside edge turning with it.
 *   - the elbow's own line ran straight into the corner (the same test on the
 *     piece below the stem) and stops there.
 *   - the other edge ENDS on the line the arm draws across the road, within
 *     XSEC_ELB_OVERRUN_ROWS. Both our lines are cut by the same painted edge.
 *     The one lookalike the drift test cannot see - a tight bend viewed from a
 *     tall mounting, whose inside edge turns inside the frame while its
 *     outside edge, on a larger radius, is still going straight up - runs on
 *     past that line without a break, which no line cut by a crossing can do.
 *
 * None of these uses the angle at the corner, which perspective makes
 * unreliable. All are measured in columns against the edge's own nearest
 * piece, so they hold at any mounting. With them in place the single-elbow
 * rule fires on none of the five circuits, none of the fault cases and none
 * of the 162 mountings - every result is byte-identical to the pair rule -
 * while the three-vector frame, both readings of it, is recognised. The
 * hand-built lookalikes in track_sim -frames, typed off the simulator's own
 * false firings, are the regression test.
 *
 * With this on, the camera's own junction report (XSEC_CAM_*) is recorded for
 * the flight recorder and no longer decides anything. Set to 0 to require a
 * matched pair, or one elbow plus the camera's agreement, as before. */
#ifndef XSEC_ELB_SINGLE
#define XSEC_ELB_SINGLE            1
#endif

/* Rows per column the arm may climb or drop and still be a crossing edge.
 * 0.60 is 31 degrees from level. The real arms measured -0.23 to +0.17 (a
 * drooping lens edge, and a car yawed twenty degrees); the chords of a bend
 * that passed everything else climbed 8 rows in 5 columns. */
#define XSEC_ELB_ARM_FLAT          0.60f

/* Rows a piece must run before its slope is trusted far enough to be
 * extrapolated the length of the stem, for the pre-bend test. Shorter pieces
 * carry too much quantisation to say where a corner should have been. */
#define XSEC_ELB_FAR_DY            4.0f

/* Columns the rest of an edge may stray from the straight line through its
 * stem, toward the arm's side, before it counts as bending that way. Measured
 * in columns of drift rather than as a change of slope so a two-row piece
 * cannot be thrown out for being too short to have a slope, and so the half a
 * column of quantisation at each end of a short piece cannot masquerade as a
 * bend. The lookalikes drift 4 to 18 columns; a straight edge cut in pieces
 * drifts under one. */
#define XSEC_ELB_SWEEP_COLS        3.0f

/* Cells apart two vector ends may be and still be one line carrying on -
 * tighter than XSEC_ELB_JOIN on purpose. This is what tells our line resuming
 * beyond the crossing, a separate piece with the width of the other track's
 * bare floor between it and the stem, from an edge that simply continues. */
#define XSEC_ELB_CHAIN             1.5f

/* Rows the other edge may run on above the line the arm draws across the
 * road. Covers the lens bowing the arm and the camera stopping a vector short
 * of the paint; the lookalike it exists for overran by 17 rows. */
#define XSEC_ELB_OVERRUN_ROWS      6.0f

/* Row at or above which a line counts as running out of the top of the frame
 * rather than ending. The other edge leaving the frame above the crossing
 * line was not cut by that line - and measured only to where the frame cuts
 * it, a corner sitting in the top few rows could never overrun by more than
 * those few rows. */
#define XSEC_ELB_TOPCLIP           1.5f

/* Row a LONE corner must sit below before it is believed. A corner along the
 * top of the frame is our line leaving the frame with something level lying
 * along the edge, and the line it would have to be checked against leaves the
 * frame too; the inside edge of a tight bend seen from a tall mounting looks
 * exactly like that while its outside edge is still on its way up, and it was
 * the last lookalike standing in the mounting sweep, at rows 0 to 3. The
 * junction is back a few frames later, lower down, with everything in view -
 * the real lone-elbow frame has its corner at row 7. The pair rule is
 * unaffected: the real two-corner frame has its corners at rows 1.6 and 2.4. */
#define XSEC_ELB_MIN_ROW           3.5f

/* Heading limit for the elbow test, looser than XSEC_MAX_HEAD.
 *
 * The bar and splay tests need the road straight because a hairpin also puts
 * lines sideways in the frame. The elbow test has its own way of telling a
 * bend, and it needs the room: the three-vector frame from the real car was
 * taken yawed some twenty degrees, with the corridor sweeping 0.59 to 0.64
 * columns a row, and a car straightening up out of a bend arrives at a
 * junction like that more often than not. 0.70 keeps a tenth in hand over
 * that frame. Raising it further admitted mid-bend frames in the mounting
 * sweep that the geometry tests then had to reject. */
#define XSEC_ELB_MAX_HEAD          0.70f

/* Columns sideways per row spanned, above which a vector is taken to be lying
 * across the road rather than running along it, and is kept out of the track
 * model while a crossing is in play. See Xsec_ForTrack in intersection.h for
 * why this is not simply done in track.c.
 *
 * 2.5 sits between the two populations with a factor of two either side. Our
 * own edges run away from the camera: about one column per row, 1.2 at worst
 * in a real frame with the car yawed twenty degrees. A crossing edge lies
 * across the road, so under any mounting it is close to level - six columns
 * per row in the frame this was measured on, and flatter the nearer it gets. */
#define XSEC_TRACK_FLAT            2.5f

/* The safety interlock, and the reason a hairpin can never be mistaken for a
 * crossing: the road through it has to be near enough straight. A corner that
 * puts a line sideways in the frame fails both of these, and a corner is exactly
 * where driving straight on at full power would be unrecoverable.
 *
 * Generous enough for a crossing on a gentle bend, which is legal and does
 * happen; far tighter than any hairpin. Same units as headFar and curv. */
#define XSEC_MAX_HEAD              0.40f
#define XSEC_MAX_CURV              0.45f

/* Frames of agreement before the car will act on it. The camera runs at 60 Hz,
 * so this is a few tens of milliseconds and costs nothing in braking distance,
 * but it throws out the single-frame flukes. */
#define XSEC_CONFIRM_FRAMES        2

/* Latch when the bar has come down to here - close enough that the corridor is
 * about to break up. Also latches early if the track is lost while a crossing is
 * recognised ahead, which is the wide-crossing case. */
#define XSEC_LATCH_Y               10.0f

/* Heading limit for COMMITTING, as opposed to merely recognising.
 *
 * Recognising a crossing several rows ahead costs nothing. Committing to one
 * means driving blind on a held angle, and the held angle is only right while
 * the road stays as it was. There is a trap here worth spelling out: a crossing
 * cuts the corridor short, and a short corridor cannot show a bend, so the far
 * heading goes quiet - not because the road is straight but because the car can
 * no longer see. Approaching a bend with a junction in it, that reads as an
 * empty straight at exactly the moment the car should be turning in.
 *
 * What does NOT go quiet is the near heading: the road under the bumper is
 * still in plain view. This used to be 0.30, and the car refused to commit
 * whenever the near heading said it was turning, driving the junction on the
 * ordinary rules instead. That refusal fired on the real frames: a car
 * straightening up on the approach carries a near heading of 0.6 whether or
 * not the road bends, and once refused it went through the junction blind on
 * its last steering command, which is worse than a held one - unbounded, and
 * with the failsafe slowing it. The limit now matches the elbow test, and the
 * protection against a bend has moved to the hold itself: the share of the
 * held angle that was answering the near heading is let go of inside the
 * junction, see XSEC_HOLD_YAW_FULL. */
#define XSEC_LATCH_MAX_HEAD        0.70f

/* How far the car drives on the latch, in metres, before it insists on seeing
 * the track again. This is the safety cap: it bounds how far a false positive
 * can carry the car, so keep it a little over one crossing width. */
#define XSEC_HOLD_M                0.90f

/* How far the car may keep approaching a junction it has recognised but never
 * reached. Once a crossing is seen the car stops giving up on it just because
 * the sighting went quiet - the crossing's edges slide off the sides of the
 * frame in the last stride before it arrives - so this is what stops a stale
 * recognition living forever. */
#define XSEC_AHEAD_MAX_M           3.00f

/* How long a sighting stays worth acting on, in metres, once the crossing has
 * stopped being visible. Covers the last stride, where its edges have slid off
 * the sides of the frame but the car has not reached the gap yet. */
#define XSEC_MEMORY_M              0.60f

/* Valid rows needed to end the latch early and steer by the corridor again.
 * Deliberately modest. The held angle is only an assumption about a road nobody
 * can see, and it goes stale fastest exactly where being wrong costs most - in
 * a bend - so the car takes the first corridor good enough to steer by rather
 * than waiting for a tidy one. */
#define XSEC_RESUME_ROWS           4

/* Once through, ignore new bars over this distance. Without it the far edge of
 * the crossing just left is immediately recognised as a new one. */
#define XSEC_CLEAR_M               0.45f

/* Steering held through the latch, as a fraction of what the car was doing on
 * the way in. 1.0 holds the entry angle, which is what a crossing on a bend
 * needs; a touch under 1 lets it straighten slightly, which is the safer error
 * on the far more common square crossing. */
#define XSEC_HOLD_STEER_K          0.85f

/* How fast the held angle tracks the steering while the corridor is whole.
 *
 * The angle that gets frozen must NOT be the one from the frame the car
 * committed on. By then the crossing has already begun eating the corridor and
 * the steering has already begun reacting to that, so freezing it holds a
 * correction for a road that is not there - which is worth about 17 cm of drift
 * across a 60 cm junction. Sampling a lightly filtered value, and only while
 * the frame is still clean, holds what the car was actually doing instead.
 * At 60 Hz this is roughly an 80 ms memory: quick enough to follow a bend,
 * far too slow to pick up the two frames that matter. */
#define XSEC_HOLD_ALPHA            0.25f

/* Valid rows the corridor must show, with both lines seen, for the hold filter
 * to keep running once a crossing has been recognised. A junction can be
 * recognised two metres out, and an angle frozen back there is a second stale
 * by the time the car commits; the filter follows the steering through the
 * approach for as long as the model runs this deep, and stops the moment the
 * crossing starts to eat the corridor. */
#define XSEC_HOLD_MIN_ROWS         5

/* Near heading at which the whole of the held angle is taken to be a yaw
 * correction, and let go of inside the junction; below it, proportionally.
 *
 * The held angle is right for as long as the reason for it lasts. On a
 * crossing laid on a bend the reason is the bend, the car is following it, the
 * road under the bumper points where the car points and the near heading is
 * small: the angle is held as it is. On a car that arrived yawed - out of the
 * bend before, or after a bump - the road under the bumper points sideways,
 * the angle is undoing that, and the yaw is gone a few tenths of a metre in.
 * Held across the whole junction it keeps turning the car: on a 17 cm
 * wheelbase a modest correction held for 60 cm turns the car through most of
 * a right angle, which is the car steering off onto the other track. Same
 * units as headNear; the real frames sit at 0.6. */
#define XSEC_HOLD_YAW_FULL         0.50f

/* Metres over which that yaw share fades to nothing, from the latch. About
 * what the steering loop would have taken to unwind it with the lines in
 * view. 0 holds the angle unchanged, as before. Measured on the simulator's
 * crossing after a bend: the junction went from costing 11 cm/s to costing
 * nothing, and the known blind spot - a junction in a 1.2 m bend - kept 12 cm
 * of clearance instead of 3. */
#define XSEC_HOLD_DECAY_M          0.35f

/* Speed through a recognised crossing, as a fraction of what the approach had
 * settled on. The whole point of the feature - 1.0 keeps everything there was. */
#define XSEC_SPEED_FRAC            1.0f

/* Corner severity above which a junction stops buying the car any speed at all.
 *
 * On the approach the car normally discounts the corridor getting shorter,
 * because a junction is a known reason for it rather than a real loss of sight.
 * That discount is only safe while the road is straight. Once the corner cues
 * agree there is a genuine bend coming, the ordinary speed rules go back to
 * having the final say, and the junction waits its turn - a car that is about
 * to have to turn is the last one that should be given its lift back. */
#define XSEC_KEEP_POWER_SEV        0.25f

/* Full power through a junction, rather than merely not lifting for one.
 *
 * XSEC_SPEED_FRAC above holds whatever speed the approach had settled on. That
 * is the conservative reading of the feature - it stops the car throwing speed
 * away for a corridor that is short because the paint stops, not because the
 * car has gone blind - but it still carries whatever the approach happened to
 * be carrying, including a lift taken for something that turned out to be the
 * junction itself.
 *
 * With this set, a recognised crossing is driven at SPEED_MAX outright. It is
 * the right thing to ask for and it is also the least defensible thing in the
 * module, so it is worth being exact about what is holding the car up:
 *
 *   - it needs a crossing recognised, which needs the road straight on all
 *     three of headNear, headFar and curv (XSEC_MAX_HEAD, XSEC_MAX_CURV)
 *   - on the approach it additionally needs the ordinary corner cue to be
 *     quiet (XSEC_KEEP_POWER_SEV). The moment the cues agree a bend is coming,
 *     the ordinary speed plan takes the power straight back
 *   - committing needs the road under the bumper straight (XSEC_LATCH_MAX_HEAD),
 *     which is the one cue a crossing cannot silence
 *   - and it is over in XSEC_HOLD_M metres whatever happens
 *
 * So the exposure is one crossing width of full throttle on a road three
 * separate cues agree is straight. Set to 0 to go back to holding the approach
 * speed, which is the same A/B the rest of the module is built for. */
#ifndef XSEC_FULL_POWER
#define XSEC_FULL_POWER            1
#endif

/* ...and the same on the APPROACH to one, which is a different question and
 * gets a different answer.
 *
 * Committing to a crossing is bounded: the car is on a held angle for
 * XSEC_HOLD_M and then the corridor decides again. Merely recognising one
 * ahead is not bounded by anything except being wrong, and the bar test is
 * wrong more often than it looks. Drawn the way a real camera draws a frame -
 * one long vector per straight piece of paint - the two edges of an ordinary
 * corner flatten out at the top of the frame and leave a gap between their
 * inner ends of 0.6 to 0.9 corridor widths, which is what a crossing leaves.
 * That fires 62 times over the five clean circuits, none of them anywhere
 * near a junction. None of them commits - the latch and the corner cues hold -
 * but with full power on the approach each one puts the car to SPEED_MAX
 * until the corner cue catches up, and the car surges and brakes its way
 * round a lap.
 *
 * Off, a false approach costs nothing: the three cues a junction corrupts are
 * stood down, which is all the feature ever claimed to need. */
#ifndef XSEC_FULL_POWER_AHEAD
#define XSEC_FULL_POWER_AHEAD      0
#endif

/* Set to 0 to build the firmware with intersection handling compiled out.
 * Worth doing once on a real track: everything else is unchanged, so it is a
 * clean A/B of what the feature is actually worth on your layout. */
#ifndef XSEC_ENABLE
#define XSEC_ENABLE                1
#endif

/* Ask the Pixy2 for its own intersection blocks alongside the vectors.
 *
 * The camera runs a junction detector of its own over the whole image, which is
 * strictly more information than the dozen vectors that survive to the driver.
 * It costs a few bytes of I2C on the frames where it finds something.
 *
 * It is recorded, not acted on. There is no Pixy2 firmware inside the host
 * simulator, so nothing here can be tested the way the geometric detector has
 * been, and an untested cue that can stop the car braking has no business
 * steering it.
 *
 * OFF by default, for two reasons that only became clear once the geometry
 * could stand on its own.
 *
 * It cannot decide anything as things stand. cam_backs() is consulted in one
 * place: to unlock a lone elbow when XSEC_ELB_SINGLE is 0. With it at 1 the
 * geometry settles that case itself and the camera is never asked, so the
 * block is pure cost.
 *
 * And the cost lands in the worst place. The block adds six bytes plus four
 * per branch to the payload, and it arrives on exactly the frame a junction is
 * in view - which is already the busiest frame there is, because a junction
 * near a corner leaves both our lines in pieces and adds the crossing's stubs.
 * At 100 kHz those bytes are the difference between a frame that arrives and
 * one that does not: see the note on the transfer timeout in pixy.c.
 *
 * Turn it back on together with XSEC_ELB_SINGLE=0 if you want the camera to be
 * the tie-breaker for a lone elbow again. Note also that the Pixy2 reports
 * each intersection once, not on every frame it is visible, so the block is
 * present far less often than a junction is. */
#ifndef PIXY_WANT_INTERSECTIONS
#define PIXY_WANT_INTERSECTIONS    0
#endif

/* =====================================================================
 * ONE-SIDED RECOVERY - finding the black line that left the frame
 * See recover.h for what this is and why the near heading gates it.
 * ===================================================================*/
/* Consecutive frames with only one edge measured before the probe arms.
 *
 * One-sided vision is not rare and is not by itself a problem: a corridor built
 * from one edge and the width model is accurate for a good fraction of a second,
 * and on a well aimed camera most one-sided stretches are shorter than that. Arm
 * too early and the car goes looking for a line that was about to reappear on its
 * own, which costs more than it buys. At ~50 frames/s this is about a fifth of a
 * second of being blind down one side. */
#ifndef RCV_ARM_FRAMES
#define RCV_ARM_FRAMES             12
#endif

/* Rows the corridor must have before any of this is worth doing. A two row
 * corridor barely defines a heading, let alone justifies steering off it. */
#ifndef RCV_MIN_ROWS
#define RCV_MIN_ROWS               4
#endif

/* |headNear| above which the probe refuses to arm, and stands down if it has
 * already armed. Deliberately the same number as CORNER_HEAD_IGNORE: this is the
 * firmware's existing definition of "the road ahead is not bending", and the
 * probe has no business disagreeing with the speed planner about that. */
#ifndef RCV_MAX_HEAD
#define RCV_MAX_HEAD               0.45f
#endif

/* Frames for the probe to go from straightening to leaning. */
#ifndef RCV_RAMP_FRAMES
#define RCV_RAMP_FRAMES            6
#endif

/* How long a probe may run before it is written off. Long enough for the yaw to
 * actually move the picture - a few tenths of a second - and short enough that
 * being wrong about a corner costs a fraction of a car length rather than a
 * corner's worth of steering. */
#ifndef RCV_MAX_FRAMES
#define RCV_MAX_FRAMES             30
#endif

/* Frames after a failed probe before another may arm. Both this and the corridor
 * being measured on both sides again are required, so a hairpin gets at most one
 * probe rather than a series of them. */
#ifndef RCV_COOL_FRAMES
#define RCV_COOL_FRAMES            20
#endif

/* How far the aim point leans toward the missing side at full ramp, as a
 * fraction of the usable half-corridor.
 *
 * Small on purpose. The lean is only there to yaw the camera, and yaw is what
 * brings a line back into a narrow frame - it does not have to move the car far
 * to do it. It is also a lean toward an edge whose position is a guess, which is
 * the thing racing_line.c refuses to do for the sake of a racing line; the
 * difference here is that this one buys the measurement back, and it is bounded,
 * timed, and still passes through the hard safety clamp underneath. */
#ifndef RCV_NUDGE_FRAC
#define RCV_NUDGE_FRAC             0.25f
#endif

/* |headFar| above which the road counts as bending for the purpose of deciding
 * which side of it the missing line is on. Below this there is no bend to be on
 * the outside of, so the lean is allowed in either direction. Same threshold as
 * a chicane is allowed to reach without being steered for. */
#ifndef RCV_BEND_HEAD
#define RCV_BEND_HEAD              0.50f
#endif

/* How much of the racing line to give up while probing, 1.0 = all of it.
 *
 * With one edge inferred, the apex and entry biases are computed from a headFar
 * that is really just the slope of the one line the car can see. Committing to a
 * racing line on that is how the car ends up leaning on a corner that was never
 * measured, so the whole bias goes while the probe runs and comes back the moment
 * the corridor does. */
#ifndef RCV_BIAS_CUT
#define RCV_BIAS_CUT               1.0f
#endif

/* Set to 0 to build with one-sided recovery compiled out. Everything else is
 * unchanged, so it is a clean A/B of what the feature is worth on your layout. */
#ifndef RCV_ENABLE
#define RCV_ENABLE                 1
#endif

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
#endif

/* Extra look-ahead rows used when the curve has been chorded, to put the aim point
 * on the far end of the chord where it touches the real line again. */
#ifndef LINE_LA_LOWCONF_BOOST
#define LINE_LA_LOWCONF_BOOST      2.0f
#endif

/*
 * ...and the exception, which on real hardware is most of the lap.
 *
 * All of the above is an argument about curves. A chord is a bad description of a
 * curve; it is a perfect description of a straight, because a straight IS its own
 * chord. A real Pixy2 pointed down an empty straight returns exactly two vectors -
 * one per edge, merged end to end - and that is not a thin frame, it is the whole
 * road. Requiring four there would hold the racing line back and push the aim point
 * out for no reason on the fastest part of the lap.
 *
 * So on a straight two vectors is full confidence. Which puts all the weight on the
 * test for "straight", because the failure it has to survive is a corner merged into
 * one chord per edge - the same two vectors, and the case finding 6.2 measures as a
 * total loss. That corner reports headNear and headFar equal, so its curv is exactly
 * zero: any test resting on curvature passes it. What does separate them:
 *
 *   headFar    a real straight sits near zero (measured ~0.05, and 0.000 dead on).
 *              A chord across a bend carries that bend's average slope, and by the
 *              time the corner is close enough to matter it is well past 0.15.
 *   both edges A corner tight enough to be dangerous pushes its inside edge out of
 *              a 60-degree frame. A straight shows both, all the way up.
 *   rows       Same argument, measured as how far the corridor reaches.
 *
 * Every one of those has to hold. Anything else keeps the four-vector requirement,
 * comes out short of full confidence, and is held below full power by
 * SPEED_CONF_FLOOR - which is the half of this that keeps the car on the road.
 */
#ifndef LINE_CONF_SEGS_STRAIGHT
#define LINE_CONF_SEGS_STRAIGHT    2    /* ...but on a straight, this many is */
#endif
#ifndef LINE_CONF_STRAIGHT_HEAD
#define LINE_CONF_STRAIGHT_HEAD    0.15f /* well under CORNER_HEAD_IGNORE 0.45 */
#endif
#ifndef LINE_CONF_STRAIGHT_CURV
#define LINE_CONF_STRAIGHT_CURV    0.30f /* belt: a chorded corner reads 0 here */
#endif
#ifndef LINE_CONF_STRAIGHT_ROWS
#define LINE_CONF_STRAIGHT_ROWS    6    /* corridor has to reach this far up   */
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
/*
 * Full power needs a corner the camera has actually described.
 *
 * Every corner cue the planner uses - headFar, curv, the steering command - is
 * computed from the vectors in this frame, so when the frame carries no curvature
 * the cues do not report a mild corner, they report no corner: a bend merged into
 * one chord per edge arrives as a straight at an angle, scores no severity at all,
 * and is driven at the speed of the straight it is pretending to be. Nothing built
 * on the corner cues can catch that, because the cues are what has gone blind.
 *
 * So this is the one limit that is not asked of the cues. It is a ceiling rather
 * than a reduction, and the difference matters: a corner the cues DID see has
 * already been slowed below it and is not touched at all, so this cannot tax a
 * corner twice or brake one that was being driven correctly. All it does is refuse
 * to carry straight line speed into a bend nobody measured.
 *
 * It is also deliberately narrow - a bend, with no curvature measurable anywhere in
 * the frame, and never a straight. Same units as SPEED_MAX and SPEED_MIN.
 *
 * And it sits BELOW SPEED_MIN, which looks wrong until you read what SPEED_MIN is:
 * the slowest the planner will go for the worst corner it can SEE. A bend with no
 * curvature in it has not been seen, only inferred from the fact that a road is
 * there, so it is not in that family and the floor built for it does not apply.
 * Measured over the 18 in-envelope camera mountings with one vector per edge, the
 * worst clearance goes -37.96 cm -> -5.51 cm on the way through 72, and is flat from
 * there down to 58; above 72 it does nothing at all. 65 is the middle of that band.
 */
#ifndef SPEED_CHORD_CEIL
#define SPEED_CHORD_CEIL           65.0f
#endif

/* Consecutive frames of no measurable curvature before the ceiling applies.
 *
 * A corner the camera is describing properly still throws the odd frame where the
 * pieces happen to collapse onto one vector per edge, and acting on one of those is
 * a brake pulse in the middle of a corner that was going fine - which perturbs the
 * line more than the speed helps. The case worth braking for is not a frame, it is a
 * bend the camera has failed to describe for as long as the car has been looking at
 * it, and that lasts hundreds of milliseconds. Release is immediate: one frame with
 * a bend measured in it is the corner coming back, and the power goes with it. */
#ifndef SPEED_CHORD_FRAMES
#define SPEED_CHORD_FRAMES         3
#endif

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

/* ---- the camera's own junction detector, as a tie-breaker -----------------
 *
 * The Pixy2 runs an intersection finder over the WHOLE image, which is strictly
 * more than the dozen vectors that reach the firmware. It can therefore see a
 * junction whose far corner never became a vector - and that is exactly the case
 * the geometry cannot settle alone (see XSEC_ELB_SINGLE, and the 9037 excursions
 * that come of guessing at it).
 *
 * So the camera gets a vote, but only ever to unlock a lone elbow the geometry
 * has already found and refused. It is never a trigger on its own: no elbow, no
 * junction, whatever the camera says.
 *
 * NOT exercised by the host simulator - there is no Pixy2 firmware in it, so
 * Xsec_CameraHint is never called there and every simulated result is the
 * geometry alone. That is deliberate: it means turning this on cannot have moved
 * any of the numbers the tuning was chosen against. It also means the camera's
 * half is only as good as your PixyMon settings, so verify before racing:
 * the flight recorder logs xsec.camAgreed, and "Intersection filtering" in
 * PixyMon's line-tracking Expert tab trades false positives against latency -
 * raise it if the camera flags junctions that are not there. */

/* Grid cells the camera's reported junction may sit from our corner and still
 * be talking about the same thing. The 79 x 52 grid is coarse and the camera
 * reports the junction centre while we hold one of its corners, so this is
 * deliberately roomy - about a third of the frame width. */
#ifndef XSEC_CAM_NEAR
#define XSEC_CAM_NEAR              25.0f
#endif

/* Branches the camera must see leaving the junction. Three is a T; a crossing
 * is four. Two would be a bend, which is not a junction at all. */
#ifndef XSEC_CAM_MIN_BRANCH
#define XSEC_CAM_MIN_BRANCH        3
/* With XSEC_ELB_SINGLE on, neither of the two above decides anything: the
 * camera's verdict is recorded (TLM_NV_CAMXSEC) and the geometry rules. The
 * PixyMon frames do draw an intersection circle on every elbow, so the camera
 * evidently flags them; how many branches it reports for an L-corner is not
 * known, and 3 may be one too many. Check TLM_NV_CAMXSEC against TLM_F_XSEC
 * on a capture before ever making the camera the deciding vote again. */
#endif
