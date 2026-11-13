#ifndef BLIND_BOX_H
#define BLIND_BOX_H
#include <stdint.h>
#include <string.h>
#include "blind_box_config.h"
typedef enum { BB_WAIT, BB_FWD, BB_STOP1, BB_REVERSE, BB_STOP2, BB_DONE, BB_FAULT, BB_CAL } bb_phase_t;
typedef enum { BB_OK, BB_CONFIG, BB_HISTORY, BB_BOUNDS, BB_DRIFT, BB_DIRECTION, BB_STALL, BB_TIMEOUT, BB_CAMERA, BB_ALIGNMENT } bb_fault_t;
typedef struct { uint32_t odo; float steer; } bb_sample_t;
typedef struct {
    bb_phase_t phase; bb_fault_t fault;
    uint8_t passes, first_stop, final_stop, ref_valid, initialized;
    uint32_t ref_odo, started_ms, still_ms, motion_ms, last_odo;
    uint32_t direction_ms;
    uint8_t direction_pending;
    int32_t x, forward_target, reverse_target, differential;
    float request;
    bb_sample_t history[BB_HISTORY_SIZE];
    unsigned head, count;
} blind_box_t;
static int32_t bb_abs(int32_t x) { return x < 0 ? -x : x; }
static int32_t bb_counts(int32_t mm) { return mm * BB_COUNTS_PER_METER / 1000; }
static int32_t bb_mm(int32_t counts) { return counts * 1000 / BB_COUNTS_PER_METER; }
static const char *bb_name(bb_phase_t p) {
    static const char * const names[] = {"WAIT","FWD","STOP1","REV","STOP2","DONE","FAULT","CAL"};
    return (unsigned)p < 8 ? names[p] : "?";
}
static int bb_active(const blind_box_t *b) { return b->phase >= BB_FWD && b->phase <= BB_FAULT; }
static void bb_fail(blind_box_t *b, bb_fault_t f) { b->fault=f; b->phase=BB_FAULT; b->request=0; }
/* 原有终点停车触发后，按两轮平均编码器里程倒车。 */
static void bb_update(blind_box_t *b,int finish,uint8_t passes,uint32_t now,uint32_t odo,
                      int left,int right,int32_t wheel_difference) {
    b->passes=passes;
    if (!BB_ENABLED) return;
    if (b->phase==BB_WAIT && finish) {
        b->phase=BB_STOP1;
        b->started_ms=now;
        b->reverse_target=-bb_counts(BB_REVERSE_DISTANCE_MM);
    }
    if (!bb_active(b) || b->phase==BB_DONE || b->phase==BB_FAULT) return;
    if (now-b->started_ms>BB_TIMEOUT_MS) { bb_fail(b,BB_TIMEOUT); return; }
    if (b->ref_valid) b->x=(int32_t)(odo-b->ref_odo);
    if (b->phase==BB_REVERSE) {
        /* 单次起步回弹不锁停，持续正向反馈仍立即结束倒车。 */
        if (left>BB_STOP_COUNTS || right>BB_STOP_COUNTS) {
            if (!b->direction_pending) {
                b->direction_pending=1;
                b->direction_ms=now;
            } else if (now-b->direction_ms>=40u) {
                bb_fail(b,BB_DIRECTION); return;
            }
        } else {
            b->direction_pending=0;
        }
        if (bb_abs(wheel_difference-b->differential)>bb_counts(BB_DIFF_LIMIT_MM)) {
            bb_fail(b,BB_DRIFT); return;
        }
        if ((int32_t)(b->last_odo-odo)>=bb_counts(2)) {
            b->last_odo=odo;
            b->motion_ms=now;
        }
        if (now-b->motion_ms>BB_STALL_MS) { bb_fail(b,BB_STALL); return; }
        if (b->x<=b->reverse_target) { b->phase=BB_STOP2; b->still_ms=0; }
    }
    if (b->phase==BB_STOP1 || b->phase==BB_STOP2) {
        if (bb_abs(left)>BB_STOP_COUNTS || bb_abs(right)>BB_STOP_COUNTS) b->still_ms=0;
        else if (!b->still_ms) b->still_ms=now;
        else if (now-b->still_ms>=BB_STOP_DWELL_MS) {
            if (b->phase==BB_STOP1) {
                b->first_stop=1;
                b->ref_valid=1;
                b->ref_odo=b->last_odo=odo;
                b->motion_ms=now;
                b->differential=wheel_difference;
                b->phase=BB_REVERSE;
            } else {
                b->final_stop=1;
                b->phase=BB_DONE;
            }
        }
    }
    b->request=b->phase==BB_REVERSE?-BB_REVERSE_SPEED:0.0f;
}
#endif
