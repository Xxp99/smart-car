void abc_element_update(track_detector_t *state, track_element_t candidate)
{
    int is_ring = candidate == TRACK_ROUND_LEFT || candidate == TRACK_ROUND_RIGHT;
    // NONE/UNKNOWN只让证据自然过期；十字或反侧环岛立即清除旧证据。
    if (candidate == TRACK_CROSS || (is_ring && candidate != state->ring_side))
        state->ring_history = 0;
    if (is_ring) state->ring_side = candidate;
    state->ring_history = (uint8_t)(((state->ring_history << 1) | (is_ring ? 1 : 0)) &
                                   ((1u << ELEMENT_RING_WINDOW) - 1u));
    state->ring_votes = 0;
    for (uint8_t bits = state->ring_history; bits; bits >>= 1)
        state->ring_votes += bits & 1u;
    if (candidate == state->candidate)
    {
        if (state->confirm_count < ELEMENT_CONFIRM_FRAMES)
        {
            ++state->confirm_count;
        }
    }
    else
    {
        state->candidate = candidate;
        state->confirm_count = 1;
    }
    if (candidate == state->confirmed)
    {
        state->release_count = 0;
    }
    else
    {
        if (state->release_count < ELEMENT_RELEASE_FRAMES)
        {
            ++state->release_count;
        }
        if (state->release_count >= ELEMENT_RELEASE_FRAMES)
        {
            state->confirmed = TRACK_NONE;
        }
    }
    // 必须当前帧也命中，不能仅靠历史在漏检帧触发确认。
    if ((is_ring && state->ring_votes >= ELEMENT_CONFIRM_FRAMES) ||
        (candidate == TRACK_CROSS && state->confirm_count >= ELEMENT_CONFIRM_FRAMES))
    {
        state->confirmed = candidate;
        state->release_count = 0;
    }
}


static int abc_nav_clip(int value, int low, int high)
{
    return value < low ? low : (value > high ? high : value);
}

static int abc_nav_pair(uint8_t left, uint8_t right, uint8_t cols)
{
    return left > 0 && right < cols - 1 && right - (int)left >= cols / 10;
}

#include "ring_vision.inc"

static void abc_nav_stage(track_nav_t *nav, track_nav_phase_t phase, uint32_t now)
{
    nav->phase = phase;
    nav->stage_ms = now;
    nav->feature_frames = 0;
    nav->straight_frames = 0;
}

// 十字：只连接上下真实边线之间的缺口，不把整片丢线当成可用路径。
static void abc_nav_bridge(uint8_t *edge, int rows, int cols, int is_left)
{
    for (int y = rows / 4; y < rows; )
    {
        int missing = is_left ? edge[y] == 0 : edge[y] == cols - 1;
        if (!missing) { ++y; continue; }
        int first = y;
        while (y < rows && (is_left ? edge[y] == 0 : edge[y] == cols - 1)) ++y;
        int top = first - 1;
        if (top < rows / 4 || y >= rows || y - first > rows / 2) continue;
        for (int k = first; k < y; ++k)
        {
            edge[k] = (uint8_t)abc_nav_clip(edge[top] +
                ((int)edge[y] - edge[top]) * (k - top) / (y - top), 1, cols - 2);
        }
    }
}

void abc_track_nav_update(track_nav_t *nav, const track_detector_t *detector,
                      const uint8_t *raw_left, const uint8_t *raw_right,
                      uint8_t *left, uint8_t *right, uint8_t *mid,
                      uint8_t rows, uint8_t cols, uint32_t now_ms)
{
    int near = rows * 4 / 5, far = rows * 3 / 5;
    int left_open = 0, right_open = 0, both_open = 0;
    int full = abc_nav_pair(raw_left[near], raw_right[near], cols) &&
               abc_nav_pair(raw_left[far], raw_right[far], cols);
    int near_mid = ((int)raw_left[near] + raw_right[near]) / 2;
    int far_mid = ((int)raw_left[far] + raw_right[far]) / 2;
    int straight = full && abc_nav_clip(near_mid - far_mid, -cols / 16, cols / 16) ==
                                     near_mid - far_mid;
    for (int y = 0; y < rows; ++y)
    {
        left[y] = raw_left[y]; right[y] = raw_right[y];
        nav->path_valid[y] = abc_nav_pair(left[y], right[y], cols);
        mid[y] = (uint8_t)(((int)left[y] + right[y]) / 2);
        // 只在普通赛道学习逐行宽度，进入特殊元素后冻结，防止开口污染。
        int width_now = (int)right[y] - left[y];
        int width_spatial = y >= 3 && y + 3 < rows &&
            abc_nav_pair(raw_left[y-3],raw_right[y-3],cols) &&
            abc_nav_pair(raw_left[y+3],raw_right[y+3],cols) &&
            abs(width_now - ((int)raw_right[y-3]-raw_left[y-3])) <= cols / 12 &&
            abs(width_now - ((int)raw_right[y+3]-raw_left[y+3])) <= cols / 12;
        if ((nav->phase == NAV_NORMAL || nav->phase == NAV_SLOW) &&
            detector->confirmed == TRACK_NONE && detector->candidate != TRACK_CROSS &&
            detector->candidate != TRACK_ROUND_LEFT && detector->candidate != TRACK_ROUND_RIGHT &&
            nav->path_valid[y] && width_now >= cols / 4 &&
            width_now <= cols * 4 / 5 && width_spatial &&
            (!nav->learned[y] || abs(width_now - nav->width[y]) <= nav->width[y] / 4))
        {
            nav->width[y] = nav->learned[y] ? (3 * nav->width[y] + width_now + 2) / 4 : width_now;
            nav->learned[y] = 1;
        }
        if (y >= rows * 2 / 3 && y < rows * 9 / 10)
        {
            left_open += left[y] == 0 && right[y] > 0 && right[y] < cols - 1;
            right_open += right[y] == cols - 1 && left[y] > 0 && left[y] < cols - 1;
            both_open += left[y] == 0 && right[y] == cols - 1;
        }
    }
    if (!nav->learned[near] && nav->path_valid[far] && nav->learned[far] &&
        ((raw_left[near] == 0 && nav->vision.side[1].monotonic) ||
         (raw_right[near] == cols - 1 && nav->vision.side[0].monotonic)))
    {
        int estimate = nav->width[far] * 4 / 3;
        if (estimate >= cols / 4 && estimate <= cols * 4 / 5)
        { nav->width[near] = (uint8_t)estimate; nav->learned[near] = 1; }
    }
    if (nav->phase == NAV_NORMAL && detector->confirmed != TRACK_NONE &&
        detector->confirmed != TRACK_UNKNOWN)
    {
        nav->element = detector->confirmed;
        nav->started_ms = now_ms;
        nav->fault = !nav->learned[near] || !nav->learned[far] ?
            NAV_FAULT_WIDTH : NAV_FAULT_NONE;
        abc_nav_stage(nav, !nav->learned[near] || !nav->learned[far] ? NAV_SLOW :
            (nav->element == TRACK_CROSS ? NAV_CROSS : NAV_APPROACH), now_ms);
    }
    if (nav->phase != NAV_NORMAL && nav->phase != NAV_RECOVER && nav->phase != NAV_SLOW &&
        now_ms - nav->started_ms >= NAV_TIMEOUT_MS)
    {
        nav->fault = NAV_FAULT_TIMEOUT;
        abc_nav_stage(nav, NAV_SLOW, now_ms);
    }
    if (nav->phase == NAV_RECOVER && now_ms - nav->stage_ms >= NAV_RECOVER_TIMEOUT_MS)
    {
        nav->fault = NAV_FAULT_RECOVER;
        abc_nav_stage(nav, NAV_SLOW, now_ms);
    }
    int is_left = nav->element == TRACK_ROUND_LEFT;
    int inner_open = is_left ? left_open : right_open;
    int outer_open = is_left ? right_open : left_open;
    ring_side_t *inside_edge = &nav->vision.side[is_left ? 0 : 1];
    ring_side_t *outside_edge = &nav->vision.side[is_left ? 1 : 0];
    int feature = 0;
    if (nav->phase == NAV_APPROACH) feature = inner_open >= rows / 20;
    if (nav->phase == NAV_ENTER) feature = full && inner_open == 0;
    if (nav->phase == NAV_INSIDE) feature = outer_open >= rows / 20 && both_open == 0;
    if (nav->vision.enabled) {
        if (nav->phase == NAV_APPROACH)
            feature = (!inside_edge->a.valid && inside_edge->b.valid &&
                       inside_edge->c.valid && inside_edge->c.y >= rows / 3) ||
                      (inside_edge->a.valid && inside_edge->a.y >= rows * 3 / 4 &&
                       (is_left ? inside_edge->a.x < cols / 4 :
                                  inside_edge->a.x > cols * 3 / 4));
        if (nav->phase == NAV_ENTER)
            feature = !inside_edge->c.valid && full && inner_open == 0;
        if (nav->phase == NAV_INSIDE)
            feature = outside_edge->a.valid && outer_open >= rows / 20 && both_open == 0;
    }
    if (feature) { if (nav->feature_frames < NAV_FEATURE_FRAMES) ++nav->feature_frames; }
    else nav->feature_frames = 0;
    if ((nav->phase == NAV_SLOW ? full : straight) && detector->candidate == TRACK_NONE)
    {
        if (nav->straight_frames < NAV_EXIT_STRAIGHT_FRAMES) ++nav->straight_frames;
    }
    else nav->straight_frames = 0;

    uint32_t elapsed = now_ms - nav->stage_ms;
    if (nav->phase == NAV_APPROACH &&
        (nav->feature_frames >= NAV_FEATURE_FRAMES ||
         (elapsed >= NAV_STAGE_MIN_MS && inside_edge->a.valid &&
          inside_edge->a.y >= rows * 3 / 4 &&
          (is_left ? inside_edge->a.x < cols / 4 :
                     inside_edge->a.x > cols * 3 / 4))))
        abc_nav_stage(nav, NAV_ENTER, now_ms);
    else if (nav->phase == NAV_ENTER && elapsed >= NAV_STAGE_MIN_MS &&
             nav->feature_frames >= NAV_FEATURE_FRAMES)
        abc_nav_stage(nav, NAV_INSIDE, now_ms);
    else if (nav->phase == NAV_INSIDE && elapsed >= NAV_INSIDE_MIN_MS &&
             nav->feature_frames >= NAV_FEATURE_FRAMES)
        abc_nav_stage(nav, NAV_EXIT, now_ms);
    else if ((nav->phase == NAV_EXIT || nav->phase == NAV_CROSS) &&
             elapsed >= NAV_STAGE_MIN_MS && nav->straight_frames >= NAV_EXIT_STRAIGHT_FRAMES)
        abc_nav_stage(nav, NAV_RECOVER, now_ms);
    else if ((nav->phase == NAV_RECOVER || nav->phase == NAV_SLOW) && elapsed >= NAV_RECOVER_MS &&
             nav->straight_frames >= NAV_EXIT_STRAIGHT_FRAMES && detector->confirmed == TRACK_NONE)
    {
        abc_nav_stage(nav, NAV_NORMAL, now_ms);
        nav->fault = NAV_FAULT_NONE;
        nav->element = TRACK_NONE;
    }

    if (nav->phase == NAV_CROSS)
    {
        abc_nav_bridge(left, rows, cols, 1);
        abc_nav_bridge(right, rows, cols, 0);
    }
    uint8_t visual_patched[256] = {0};
    if (nav->vision.enabled) {
        uint8_t *edge = is_left ? left : right;
        ring_point_t a = {0}, b = {0};
        if (nav->phase == NAV_APPROACH) {
            a = inside_edge->a; b = inside_edge->b;
            if (!a.valid && b.valid) {
                a.y = rows - 1;
                a.x = is_left ? raw_left[a.y] : raw_right[a.y];
                a.valid = a.x > 0 && a.x < cols - 1;
            }
        }
        // C directs the opposite boundary into the ring (mirrored V-line).
        if (nav->phase == NAV_ENTER && inside_edge->c.valid) {
            edge = is_left ? right : left;
            b = inside_edge->c;
            a.y = rows - 1; // cover the 4/5-height steering row, not just the far field
            a.x = is_left ? raw_right[a.y] : raw_left[a.y];
            a.valid = a.x > 0 && a.x < cols - 1;
        }
        if (nav->phase == NAV_EXIT && outside_edge->a.valid) {
            edge = is_left ? right : left;
            b = outside_edge->a;
            a.y = rows - 1;
            a.x = is_left ? raw_right[a.y] : raw_left[a.y];
            a.valid = a.x > 0 && a.x < cols - 1;
        }
        if (a.valid && b.valid && a.y != b.y) {
            ring_join(edge, a, b, cols);
            for (int y = a.y < b.y ? a.y : b.y; y <= (a.y > b.y ? a.y : b.y); ++y)
                visual_patched[y] = 1;
            if (nav->phase == NAV_ENTER && b.y > rows / 8) {
                ring_point_t far_anchor;
                far_anchor.y = rows / 8;
                far_anchor.x = is_left ? cols / 3 : cols * 2 / 3;
                far_anchor.valid = 1;
                ring_join(edge, far_anchor, b, cols);
                for (int y = far_anchor.y; y <= b.y; ++y) visual_patched[y] = 1;
            }
        }
    }
    for (int y = rows / 4; y < rows; ++y)
    {
        if (visual_patched[y] && nav->learned[y]) {
            int patched_left = nav->phase == NAV_APPROACH ? is_left : !is_left;
            if (patched_left)
                right[y] = (uint8_t)abc_nav_clip(left[y] + nav->width[y], 1, cols - 2);
            else
                left[y] = (uint8_t)abc_nav_clip(right[y] - nav->width[y], 1, cols - 2);
        }
        int usable = abc_nav_pair(left[y], right[y], cols);
        int center = ((int)left[y] + right[y]) / 2;
        int width = nav->width[y];
        int ring = nav->phase == NAV_APPROACH || nav->phase == NAV_ENTER ||
                   nav->phase == NAV_INSIDE || nav->phase == NAV_EXIT;
        if (ring && nav->learned[y] && !visual_patched[y])
        {
            int inner_valid = is_left ? raw_left[y] > 0 && raw_left[y] < cols - 1 :
                                       raw_right[y] > 0 && raw_right[y] < cols - 1;
            int outer_valid = is_left ? raw_right[y] > 0 && raw_right[y] < cols - 1 :
                                       raw_left[y] > 0 && raw_left[y] < cols - 1;
            if (raw_left[y] >= raw_right[y]) inner_valid = outer_valid = 0;
            usable = 0;
            // 入口沿外侧边线，入环时向开口侧偏移；环内/出口优先沿内侧岛边。
            if ((nav->phase == NAV_INSIDE || nav->phase == NAV_EXIT) && inner_valid)
            {
                center = is_left ? raw_left[y] + width / 2 : raw_right[y] - width / 2;
                usable = 1;
            }
            else if (outer_valid)
            {
                center = is_left ? raw_right[y] - width / 2 : raw_left[y] + width / 2;
                usable = 1;
            }
            else if (nav->phase == NAV_ENTER && inner_valid)
            {
                // 入环时外侧边线也可能暂时出画面，沿仍可见的内侧边线补路宽。
                center = is_left ? raw_left[y] + width / 2 : raw_right[y] - width / 2;
                usable = 1;
            }
            if (usable && nav->phase == NAV_ENTER)
                center += (is_left ? -1 : 1) * width * NAV_ENTRY_BIAS_PERCENT / 100;
            if (usable)
            {
                left[y] = (uint8_t)abc_nav_clip(center - width / 2, 1, cols - 2);
                right[y] = (uint8_t)abc_nav_clip(center + width / 2, 1, cols - 2);
                usable = abc_nav_pair(left[y], right[y], cols);
            }
        }
        // 十字/退出恢复时的单边丢线，用进入前学习的该行宽度补另一侧。
        if (!usable && nav->learned[y] &&
            (nav->phase == NAV_CROSS || nav->phase == NAV_RECOVER || nav->phase == NAV_SLOW))
        {
            if (left[y] == 0 && right[y] > 0 && right[y] < cols - 1)
                left[y] = (uint8_t)abc_nav_clip(right[y] - width, 1, cols - 2);
            else if (right[y] == cols - 1 && left[y] > 0 && left[y] < cols - 1)
                right[y] = (uint8_t)abc_nav_clip(left[y] + width, 1, cols - 2);
            usable = abc_nav_pair(left[y], right[y], cols);
            center = ((int)left[y] + right[y]) / 2;
        }
        // 真实/补线可用性与短时保持分开，保持不能刷新last_path_ms。
        nav->path_valid[y] = usable;
        if (usable)
        {
            if (!ring) center = ((int)left[y] + right[y]) / 2;
            if (ring && nav->last_mid[y] && now_ms - nav->last_path_ms <= NAV_HOLD_MS)
                center = abc_nav_clip(center, (int)nav->last_mid[y]-8,
                                               (int)nav->last_mid[y]+8);
            mid[y] = (uint8_t)abc_nav_clip(center, 1, cols - 2);
            nav->last_mid[y] = mid[y];
        }
        else if (nav->phase != NAV_NORMAL && nav->learned[y] &&
                 now_ms - nav->last_path_ms <= NAV_HOLD_MS)
            mid[y] = nav->last_mid[y];
    }
    // 左环入口内侧边线先出画面。外侧连续且远处路径可信时，用透视宽度补近处。
    if (nav->phase == NAV_NORMAL && !nav->path_valid[near] && nav->path_valid[far] &&
        nav->learned[near] &&
        ((raw_left[near] == 0 && raw_right[near] > 0 && raw_right[near] < cols - 1 &&
          nav->vision.side[1].monotonic) ||
         (raw_right[near] == cols - 1 && raw_left[near] > 0 &&
          nav->vision.side[0].monotonic)))
    {
        int width = nav->width[near];
        if (raw_left[near] == 0 && raw_right[near] > width)
        {
            left[near] = (uint8_t)(raw_right[near] - width);
            right[near] = raw_right[near];
        }
        else if (raw_right[near] == cols - 1 && raw_left[near] + width < cols - 1)
        { left[near] = raw_left[near]; right[near] = (uint8_t)(raw_left[near] + width); }
        if (abc_nav_pair(left[near],right[near],cols))
        { mid[near] = (uint8_t)((left[near] + right[near]) / 2); nav->path_valid[near] = 1; }
    }
    if (nav->path_valid[near] && nav->path_valid[far]) nav->last_path_ms = now_ms;
    else if (nav->phase != NAV_NORMAL && nav->phase != NAV_SLOW &&
             now_ms - nav->last_path_ms > NAV_HOLD_MS)
    {
        nav->fault = NAV_FAULT_PATH;
        abc_nav_stage(nav, NAV_SLOW, now_ms);
        // 丢线退出特殊动作的当帧立即撤掉环岛偏移，回到原始中线。
        for (int y = 0; y < rows; ++y)
        {
            left[y] = raw_left[y]; right[y] = raw_right[y];
            mid[y] = (uint8_t)(((int)raw_left[y] + raw_right[y]) / 2);
        }
    }
}

