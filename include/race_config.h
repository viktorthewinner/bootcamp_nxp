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
 * with no debugger attached every PRINTF stalls the control loop. */
#define RACE_DEBUG                 0

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
#define LINE_APEX_BIAS             0.85f   /* dive to the inside at the apex     */
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

/* Aim point. Slow = look close and be precise, fast = look far and be smooth. */
#define LINE_LA_ROW_MIN            3
#define LINE_LA_ROW_MAX            6

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

/* Heading that saturates the KH term, in pixels of x per row of y. */
#define STEER_HEAD_SCALE           1.10f

/* Servo rate limit, steering units per second. Stops the linkage from slamming. */
#define STEER_SLEW_PER_S           900.0f

/* The D term runs on this much smoothing, to keep camera noise out of the servo. */
#define STEER_D_ALPHA              0.40f

/* =====================================================================
 * SPEED - as fast as the situation allows, every single frame
 * ===================================================================*/
/* THE headline number. Start at 65, raise it 5 at a time. */
#ifndef SPEED_MAX
#define SPEED_MAX                  65.0f
#endif

/* Slowest the car is allowed to go while it can still see the track. */
#define SPEED_MIN                  32.0f

/* Global scale, handy for a quick trackside calm-down. 1.0 = full. */
#define SPEED_SCALE                1.00f

/* How strongly each cue slows the car. The largest wins, so a corner seen far ahead
 * brakes the car early even while the road right in front is still straight. */
#define SPEED_W_HEAD_FAR           1.05f
#define SPEED_W_CURV               0.85f
#define SPEED_W_STEER              0.90f

/* Corner severity that pins the car at SPEED_MIN. */
#define SPEED_SEVERITY_FULL        1.00f

/* You may only drive as fast as you can see. Speed is scaled by how far ahead the
 * track model is still valid. */
#define SPEED_SEE_ROWS_FULL        6       /* this many valid rows = no penalty */
#define SPEED_SEE_ROWS_MIN         2       /* at or below this = crawl          */
#define SPEED_SEE_FLOOR            0.45f

/* Cap while only one edge of the track is visible. */
#define SPEED_ONE_EDGE_CAP         0.82f

/* Acceleration is ramped, braking is instant - same as a real car. Units per second. */
#define SPEED_ACCEL_PER_S          140.0f
#define SPEED_ACCEL_EXIT_PER_S     260.0f  /* corner exit: get on the power hard */

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
#define DIFF_GAIN                  0.45f   /* 0 = off, 0.45 = inner wheel at 55% at full lock */

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
