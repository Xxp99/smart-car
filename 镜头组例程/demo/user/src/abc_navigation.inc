// 多帧元素确认器：维护环岛方向投票、当前候选和释放计数；只输出识别标签，
// 不负责补线或行驶阶段切换，导航控制统一由 img_process.c 中的状态机处理。
void abc_element_update(track_detector_t *state, track_element_t candidate)
{
    int is_ring = candidate == TRACK_ROUND_LEFT || candidate == TRACK_ROUND_RIGHT;

    // 十字或方向相反的新环岛出现时，清掉上一组环岛投票。
    if (candidate == TRACK_CROSS || (is_ring && candidate != state->ring_side))
        state->ring_history = 0;
    if (is_ring) state->ring_side = candidate;

    state->ring_history = (uint8_t)(((state->ring_history << 1) |
                                     (is_ring ? 1 : 0)) &
                                    ((1u << ELEMENT_RING_WINDOW) - 1u));
    state->ring_votes = 0;
    for (uint8_t bits = state->ring_history; bits; bits >>= 1)
        state->ring_votes += bits & 1u;

    if (candidate == state->candidate) {
        if (state->confirm_count < ELEMENT_CONFIRM_FRAMES)
            ++state->confirm_count;
    } else {
        state->candidate = candidate;
        state->confirm_count = 1;
    }

    if (candidate == state->confirmed) {
        state->release_count = 0;
    } else {
        if (state->release_count < ELEMENT_RELEASE_FRAMES)
            ++state->release_count;
        if (state->release_count >= ELEMENT_RELEASE_FRAMES)
            state->confirmed = TRACK_NONE;
    }

    // 只能由本帧的环岛证据参与多数确认，防止漏检帧沿用陈旧识别。
    if ((is_ring && state->ring_votes >= ELEMENT_CONFIRM_FRAMES) ||
        (candidate == TRACK_CROSS && state->confirm_count >= ELEMENT_CONFIRM_FRAMES)) {
        state->confirmed = candidate;
        state->release_count = 0;
    }
}
