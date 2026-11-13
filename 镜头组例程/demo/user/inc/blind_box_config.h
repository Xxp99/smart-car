#ifndef BLIND_BOX_CONFIG_H
#define BLIND_BOX_CONFIG_H
/* Lengths are mm; speed is encoder counts / 10 ms. -1 means UNMEASURED.
 * Calibrate before setting BB_CALIBRATED and BB_REVERSE_VERIFIED to 1.
 * Missing calibration retains the existing valid finish stop, never reverses. */
#define BB_ENABLED 1
#define BB_CALIBRATED 0
#define BB_REVERSE_VERIFIED 0
/* Explicit wheels-off-ground direction test; never enable for a race run. */
#define BB_BENCH_TEST 0
#define BB_COUNTS_PER_METER 11000
#define BB_REVERSE_DISTANCE_MM 1300
#define BB_FRONT_MM 270
#define BB_REAR_MM 30
#define BB_WIDTH_MM 200
#define BB_WHEELBASE_MM 190
#define BB_STRIPE_MM (-1)
/* Rear axle position relative to stripe FAR edge on the CLEAR transition.
 * Negative if the rear axle has not crossed that edge yet. */
#define BB_CLEAR_AXLE_FROM_FAR_MM (-10000)
#define BB_FORWARD_STRAIGHT_MM (-1)
#define BB_BACK_STRAIGHT_MM (-1)
#define BB_TRACK_WIDTH_MM (-1)
#define BB_BRAKE_FWD_MM (-1)
#define BB_BRAKE_REV_MM (-1)
/* Policy margins, NOT measured geometry. */
#define BB_GAP_MM 50
#define BB_SIDE_MARGIN_MM 30
#define BB_STOP_TOL_MM 30
#define BB_DIFF_LIMIT_MM 50
#define BB_FORWARD_SPEED 20.0f
#define BB_REVERSE_SPEED 30.0f
#define BB_STOP_COUNTS 2
#define BB_STOP_DWELL_MS 500u
#define BB_TIMEOUT_MS 30000u
#define BB_STALL_MS 1500u
#define BB_FRAME_TIMEOUT_MS 300u
#define BB_HISTORY_SIZE 256u
#define BB_HISTORY_STEP_MM 10
#endif
