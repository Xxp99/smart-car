#ifndef ZEBRA_PASS_H
#define ZEBRA_PASS_H
#include <stdint.h>

#define ZEBRA_CONFIRM_FRAMES       3u
#define ZEBRA_CLEAR_FRAMES         5u
#define ZEBRA_STRAIGHT_MIN_MS     200u
#define ZEBRA_STRAIGHT_MAX_MS    1500u
#define ZEBRA_RECOVER_MS          300u
#define ZEBRA_STABLE_PIXELS         4
#define ZEBRA_PASS_SPEED          45.0f
#define ZEBRA_STRAIGHT_ANGLE      90.0f
#define ZEBRA_EXIT_DEG_PER_SEC    20.0f
#define ZEBRA_EXIT_MAX_STEP_MS      50u
#define ZEBRA_START_GUARD_MS      2000u
#define ZEBRA_START_CLEAR_MS       800u
#define ZEBRA_FRAME_GAP_MS         300u
// 条纹消失、道路稳定后继续前进的两轮平均编码器计数；不是毫米。
// 初始试车值：目标35且实际跟随时约0.5秒，须按摄像头/车长标定。
#define ZEBRA_FINISH_CLEAR_COUNTS 1750u

/* Binary input, before overlay. Require repeated runs, not a single black bar. */
static unsigned zebra_transitions(const uint8_t *p, unsigned count, unsigned step)
{
    unsigned chain = 0, longest = 0, run = 0;
    if (!count) return 0;
    uint8_t color = p[0];
    for (unsigned i = 0; i < count; ++i)
    {
        if (p[i * step] == color) ++run;
        else
        {
            // Both black and white runs must be >=2px. A 1px run breaks the chain.
            chain = run >= 2 ? chain + 1 : 0;
            if (chain > longest) longest = chain;
            color = p[i * step];
            run = 1;
        }
    }
    chain = run >= 2 ? chain + 1 : 0;
    if (chain > longest) longest = chain;
    return longest ? longest - 1 : 0;
}

static uint8_t zebra_detect(const uint8_t *image, unsigned h, unsigned w)
{
    unsigned consecutive = 0;
    for (unsigned y = h / 3; y < h * 9 / 10; ++y)
    {
        if (zebra_transitions(image + y * w + w / 6, w * 2 / 3, 1) >= 6)
        {
            if (++consecutive >= 3) return 1;
        }
        else consecutive = 0;
    }
    /* Also accept transverse bars only when all three interior columns agree. */
    for (unsigned i = 0; i < 3; ++i)
    {
        unsigned x = (int)(w / 2) + ((int)i - 1) * (int)(w / 12);
        if (zebra_transitions(image + (h / 3) * w + x, h / 2, w) < 6) return 0;
    }
    return 1;
}

typedef enum
{
    ZEBRA_NORMAL = 0,
    ZEBRA_STRAIGHT,
    ZEBRA_RECOVER,
    ZEBRA_TIMEOUT
} zebra_phase_t;

typedef struct
{
    zebra_phase_t phase;
    uint8_t hits, clear, previous_valid;
    uint8_t previous_near, previous_far;
    uint32_t phase_ms, last_frame_ms;
    uint8_t frame_initialized;
    uint8_t path_stable, steering_ready;
    uint32_t steering_ms;
    float steering_angle, steering_target;
} zebra_pass_t;

static uint8_t zebra_row_valid(const uint8_t *l, const uint8_t *r,
                               const uint8_t *bad, unsigned y, unsigned w)
{
    return !bad[y] && l[y] > 0 && r[y] < w - 1 &&
           r[y] > l[y] && (unsigned)(r[y] - l[y]) >= w / 6;
}

static void zebra_update(zebra_pass_t *s, uint8_t candidate, uint8_t path_valid,
                         uint8_t near_mid, uint8_t far_mid, uint32_t now)
{
    if (s->frame_initialized && (uint32_t)(now-s->last_frame_ms) >= ZEBRA_FRAME_GAP_MS)
    {
        s->hits = s->clear = s->previous_valid = 0;
    }
    s->frame_initialized = 1;
    s->last_frame_ms = now;
    int dn = (int)near_mid - s->previous_near;
    int df = (int)far_mid - s->previous_far;
    uint8_t stable = path_valid && s->previous_valid &&
        dn >= -ZEBRA_STABLE_PIXELS && dn <= ZEBRA_STABLE_PIXELS &&
        df >= -ZEBRA_STABLE_PIXELS && df <= ZEBRA_STABLE_PIXELS;
    s->path_stable = !candidate && stable;
    // Candidate rows cannot be used as evidence of normal boundary recovery.
    s->previous_valid = path_valid && !candidate;
    s->previous_near = near_mid;
    s->previous_far = far_mid;
    if (!candidate && stable)
    {
        if (s->clear < ZEBRA_CLEAR_FRAMES) ++s->clear;
    }
    else s->clear = 0;

    if (s->phase == ZEBRA_NORMAL)
    {
        if (candidate)
        {
            if (s->hits < ZEBRA_CONFIRM_FRAMES) ++s->hits;
            if (s->hits >= ZEBRA_CONFIRM_FRAMES)
            {
                s->phase = ZEBRA_STRAIGHT;
                s->phase_ms = now;
                s->clear = 0;
                s->steering_angle = s->steering_target = ZEBRA_STRAIGHT_ANGLE;
                s->steering_ms = now;
                s->steering_ready = 0;
            }
        }
        else s->hits = 0;
    }
    else if (s->phase == ZEBRA_STRAIGHT)
    {
        uint32_t elapsed = (uint32_t)(now - s->phase_ms);
        if (elapsed >= ZEBRA_STRAIGHT_MIN_MS && s->clear >= ZEBRA_CLEAR_FRAMES)
        {
            s->phase = ZEBRA_RECOVER;
            s->phase_ms = now;
            s->steering_ms = now;
            s->steering_ready = 0;
        }
        else if (elapsed >= ZEBRA_STRAIGHT_MAX_MS)
        {
            // Stop forcing 90; return steering to normal tracking, still at low speed.
            s->phase = ZEBRA_TIMEOUT;
            s->phase_ms = now;
            s->steering_ms = now;
            s->steering_ready = 0;
        }
    }
    else if ((uint32_t)(now - s->phase_ms) >= ZEBRA_RECOVER_MS &&
             s->clear >= ZEBRA_CLEAR_FRAMES && s->steering_ready)
    {
        s->phase = ZEBRA_NORMAL;
        s->hits = 0;
    }
    else if (s->phase == ZEBRA_RECOVER &&
             (uint32_t)(now - s->phase_ms) >= ZEBRA_STRAIGHT_MAX_MS &&
             s->clear < ZEBRA_CLEAR_FRAMES)
    {
        // Persistent loss during recovery: stop retaining an old target indefinitely.
        // TIMEOUT still limits steering rate and speed; it does not restart GO90.
        s->phase = ZEBRA_TIMEOUT;
        s->phase_ms = now;
    }
}

static uint8_t zebra_slow(const zebra_pass_t *s, uint8_t candidate)
{
    return candidate || s->phase != ZEBRA_NORMAL;
}

static float zebra_steering(zebra_pass_t *s, float tracking_angle, uint32_t now)
{
    if (s->phase == ZEBRA_STRAIGHT)
    {
        s->steering_angle = s->steering_target = ZEBRA_STRAIGHT_ANGLE;
        s->steering_ms = now;
        s->steering_ready = 0;
        return ZEBRA_STRAIGHT_ANGLE;
    }
    if (s->phase == ZEBRA_RECOVER || s->phase == ZEBRA_TIMEOUT)
    {
        uint32_t elapsed = (uint32_t)(now - s->steering_ms);
        s->steering_ms = now;
        if (elapsed > ZEBRA_EXIT_MAX_STEP_MS) elapsed = ZEBRA_EXIT_MAX_STEP_MS;
        // RECOVER rejects a new angle on a noisy/lost/striped frame. TIMEOUT
        // resumes ordinary tracking, but never removes the per-output rate limit.
        if (s->path_stable || s->phase == ZEBRA_TIMEOUT)
            s->steering_target = tracking_angle;
        float step = ZEBRA_EXIT_DEG_PER_SEC * ((float)elapsed / 1000.0f);
        float delta = s->steering_target - s->steering_angle;
        if (delta > step) delta = step;
        if (delta < -step) delta = -step;
        s->steering_angle += delta;
        float error = tracking_angle - s->steering_angle;
        s->steering_ready = s->path_stable && error >= -0.05f && error <= 0.05f;
        return s->steering_angle;
    }
    s->steering_angle = s->steering_target = tracking_angle;
    s->steering_ms = now;
    s->steering_ready = 1;
    return tracking_angle;
}

typedef enum
{
    ZEBRA_RACE_START = 0, // Ignore the departure stripe, including repeated detections.
    ZEBRA_RACE_ARMED,
    ZEBRA_RACE_PASS,      // Return stripe confirmed: keep moving across it.
    ZEBRA_RACE_CLEAR,     // Stable clear road: measure remaining clearance.
    ZEBRA_RACE_STOP
} zebra_race_phase_t;

typedef struct
{
    zebra_race_phase_t phase;
    uint8_t initialized, clear_tracking, finish_hits;
    uint32_t start_ms, clear_ms, last_ms, clearance_start_counts;
} zebra_race_t;

static void zebra_race_update(zebra_race_t *race, const zebra_pass_t *pass,
                              uint8_t candidate, uint32_t now, uint32_t distance_counts)
{
    if (race->phase == ZEBRA_RACE_STOP) return; // Reset/power-cycle is required to restart.
    if (!race->initialized)
    {
        race->initialized = 1;
        race->start_ms = race->last_ms = now; // First processed frame, not power-on time.
    }
    if ((uint32_t)(now - race->last_ms) >= ZEBRA_FRAME_GAP_MS)
    {
        race->clear_tracking = 0;
        race->finish_hits = 0;
        // Do not count travel during a camera interruption as proof of passing.
        if (race->phase == ZEBRA_RACE_CLEAR) race->phase = ZEBRA_RACE_PASS;
    }
    race->last_ms = now;
    if (race->phase == ZEBRA_RACE_START)
    {
        // No lap distance is assumed: first allow departure, then require a
        // continuous clear, recovered road before arming the return-line stop.
        if (!candidate && pass->phase == ZEBRA_NORMAL && pass->path_stable &&
            pass->clear >= ZEBRA_CLEAR_FRAMES)
        {
            if (!race->clear_tracking)
            {
                race->clear_tracking = 1;
                race->clear_ms = now;
            }
            if ((uint32_t)(now - race->start_ms) >= ZEBRA_START_GUARD_MS &&
                (uint32_t)(now - race->clear_ms) >= ZEBRA_START_CLEAR_MS)
                race->phase = ZEBRA_RACE_ARMED;
        }
        else race->clear_tracking = 0;
        return;
    }
    if (race->phase == ZEBRA_RACE_ARMED)
    {
        if (candidate)
        {
            if (race->finish_hits < ZEBRA_CONFIRM_FRAMES) ++race->finish_hits;
            if (race->finish_hits >= ZEBRA_CONFIRM_FRAMES) race->phase = ZEBRA_RACE_PASS;
        }
        else race->finish_hits = 0;
        return;
    }
    // A missed frame or lost lane alone is not evidence that the stripe was passed.
    uint8_t road_clear = !candidate && pass->phase != ZEBRA_STRAIGHT &&
        pass->path_stable && pass->clear >= ZEBRA_CLEAR_FRAMES;
    if (!road_clear)
    {
        race->phase = ZEBRA_RACE_PASS;
        return;
    }
    if (race->phase == ZEBRA_RACE_PASS)
    {
        race->phase = ZEBRA_RACE_CLEAR;
        race->clearance_start_counts = distance_counts;
        return;
    }
    // Signed difference handles counter wrap and prevents reverse travel counting as forward.
    if ((int32_t)(distance_counts-race->clearance_start_counts) >= (int32_t)ZEBRA_FINISH_CLEAR_COUNTS)
        race->phase = ZEBRA_RACE_STOP;
}

static const char *zebra_status(const zebra_pass_t *s, const zebra_race_t *race,
                               uint8_t candidate)
{
    // Equal lengths, 21*6=126px: clear old text without overflowing the 128px LCD.
    if (race->phase == ZEBRA_RACE_STOP) return "Z:STOP  RESET TO RUN ";
    if (race->phase == ZEBRA_RACE_PASS) return "Z:PASS  FINISH LINE  ";
    if (race->phase == ZEBRA_RACE_CLEAR) return "Z:CLEAR THEN STOP    ";
    if (race->phase == ZEBRA_RACE_ARMED)
        return candidate ? "Z:CHECK FINISH ARMED " : "Z:ARMED PASS THEN END";
    switch (s->phase)
    {
        case ZEBRA_STRAIGHT: return "Z:GO90  START PASS   ";
        case ZEBRA_RECOVER:  return "Z:BACK  START PASS   ";
        case ZEBRA_TIMEOUT:  return "Z:TIME  START PASS   ";
        default: return candidate ? "Z:CHECK START PASS   " : "Z:START IGNORE LINE  ";
    }
}
#endif
