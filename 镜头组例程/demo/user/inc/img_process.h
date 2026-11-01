#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#include "zf_common_headfile.h"

// 基于边线开口的初步识别，阈值需用实际赛道图像标定。
#define ELEMENT_RING_WINDOW (5)
#define NAV_ENTRY_BIAS_PERCENT (18)
#define ELEMENT_CONFIRM_FRAMES  (3)
#define ELEMENT_RELEASE_FRAMES  (5)
#define ELEMENT_GAP_MIN_DIV     (20) // 连续开口至少图像高度的1/20
#define ELEMENT_GAP_MAX_DIV     (3)  // 排除大范围丢线
#define ELEMENT_EDGE_STEP_DIV   (20) // 环岛外侧边线允许的相邻行变化
#define ROUND_EDGE_MARGIN       (2)  // 距图像边缘2像素以内也按开口检查
#define ROUND_GAP_MIN_DIV       (15) // 环岛开口最少H/15行，120行图像为8行
#define ROUND_GAP_MAX_PERCENT   (45) // 最多占图像高度45%，排除大范围单侧丢线
#define ROUND_ANCHOR_WINDOW     (5)
#define ROUND_ANCHOR_REQUIRED   (3)  // 上下各5行中至少3行恢复双边线
#define ROUND_CORNER_MIN_DIV    (10) // 入口断点距边缘至少W/10，188列时为18像素
#define ROUND_EXIT_CORNER_MIN_DIV (16) // 出口检测保持独立，不随入口收紧
#define ROUND_OUTER_VISIBLE_PERCENT (70) // 参考边线检查区至少70%的行实际可见
#define ROUND_OUTER_LOSS_MAX_DIV (20) // 每段出画最多H/20行，120行图像为6行
#define ROUND_OUTER_CLIP_MARGIN_DIV (12) // 缺口两端距对应画面边缘不超过W/12
#define ROUND_CONFIRM_FRAMES     (3) // 环岛连续3帧同向确认，减少高速通过时的确认延迟
#define ROUND_SEQUENCE_GAP_MS    (300u)
#define ROUND_SEQUENCE_MAX_MS    (1500u)

typedef enum
{
    TRACK_NONE = 0,
    TRACK_CROSS,
    TRACK_ROUND_LEFT,
    TRACK_ROUND_RIGHT,
    TRACK_UNKNOWN
} track_element_t;

typedef struct
{
    track_element_t candidate; // 当前帧，可能受噪声影响
    track_element_t confirmed; // 多帧确认结果，不是环岛行驶阶段
    uint8_t confirm_count;
    uint8_t release_count;
    uint8_t ring_history, ring_votes;
    track_element_t ring_side;
    uint8_t sequence_side, sequence_kind, sequence_row, sequence_progress;
    uint32_t sequence_start_ms, sequence_last_ms;
} track_detector_t;

// 普通道路形状：0未知、1直道、2弯道；仅用于状态显示。
uint8_t track_road_shape(const uint8_t *left, const uint8_t *right, uint8_t rows, uint8_t cols);

track_element_t track_element_classify(const uint8_t *left, const uint8_t *right,
                                      uint8_t rows, uint8_t cols);
void track_element_update(track_detector_t *state, track_element_t candidate);
void track_element_observe(track_detector_t *state,const uint8_t *left,const uint8_t *right,
                           uint8_t rows,uint8_t cols,uint32_t now_ms);
const char *track_element_name(track_element_t element);

// 视觉状态机初始参数，需按实际环岛尺寸、帧率和相机安装标定。
#define NAV_FEATURE_FRAMES       (3)
#define NAV_ENTRY_APEX_FRAMES    (2) // 已确认环岛内岛尖连续可见时允许结束入口阶段
#define NAV_EXIT_STRAIGHT_FRAMES (5)
#define NAV_RING_EXIT_CONFIRM_FRAMES (8) // 环内出口证据需累计确认，单帧/短暂开口不切换
#define NAV_RING_EXIT_STRAIGHT_FRAMES (12) // 环岛出口与恢复阶段保持稳定直线
#define NAV_RING_STRAIGHT_EXIT_MIN_MS (1200u) // 备用直线出口证据需在环内持续运行后才启用
#define NAV_STAGE_MIN_MS         (150u)
#define NAV_RING_EXIT_MIN_MS     (400u)
#define NAV_INSIDE_MIN_MS        (500u)
#define NAV_HOLD_MS              (200u)
#define NAV_INSIDE_HOLD_MS       (500u)
#define NAV_TIMEOUT_MS           (8000u)
#define NAV_RING_TIMEOUT_MS      (15000u) // 7/8日志环岛完整一圈超过8秒，环岛单独放宽超时
#define NAV_RECOVER_MS           (600u)
#define NAV_RING_RECOVER_MS      (1200u)
#define NAV_RECOVER_TIMEOUT_MS   (3000u)
#define NAV_FAR_SUPPORT_ROWS    (6) // 连续真实双边线用于远端道路方向
#define NAV_FAR_MAX_SLOPE        (2) // 每行中线最大变化像素
#define NAV_EXIT_ROW_TOLERANCE_DIV (20) // 出口道路支撑段允许离开口角点最多H/20行
#define NAV_ENTRY_SHIFT_DIV      (6) // 无岛边时入环引导偏移不超过已学习路宽的1/6

typedef enum
{
    NAV_NORMAL = 0, NAV_CROSS, NAV_APPROACH, NAV_ENTER,
    NAV_INSIDE, NAV_EXIT, NAV_RECOVER, NAV_SLOW
} track_nav_phase_t;

typedef enum
{
    NAV_FAULT_NONE = 0, NAV_FAULT_WIDTH, NAV_FAULT_TIMEOUT,
    NAV_FAULT_RECOVER, NAV_FAULT_PATH
} track_nav_fault_t;

typedef struct
{
    uint8_t x, y, valid;
} ring_point_t;

typedef struct
{
    ring_point_t a, b, c;
    uint8_t monotonic, lost;
} ring_side_t;

typedef struct
{
    uint8_t enabled;
    ring_side_t side[2]; // 0=left, 1=right, refreshed from the binary image
} ring_vision_t;

typedef struct
{
    track_nav_phase_t phase;
    track_nav_phase_t slow_resume_phase; // 环岛丢线进入SLOW前的导航阶段，用于恢复同一侧循迹
    track_nav_fault_t fault;
    ring_vision_t vision;
    track_element_t element;
    uint32_t started_ms, stage_ms, last_path_ms, apex_ms;
    uint8_t feature_frames, straight_frames, exit_confirm_frames;
    uint8_t exit_anchor, guide_valid, drive_valid, entry_pending;
    uint8_t exit_guide_row, exit_guide_center, exit_guide_valid;
    uint8_t exit_evidence, exit_straight_fallback;
    int16_t exit_guide_slope;
    uint8_t entry_guide; // 当前近控制行：0参考边回退，1开口引导，2真实岛边
    uint8_t entry_gap_valid, entry_gap_rows, entry_edge_rows, entry_guided_rows;
    uint8_t entry_full, entry_turn, entry_pair_support, entry_apex_frames, entry_bias_active, entry_transition_reason;
    uint8_t exit_outer_open, exit_both_open, exit_straight;
    int16_t entry_bend_delta;
    uint8_t exit_reject_rows;
    uint8_t apex_valid, apex_x, apex_y, apex_hits, apex_prev_x, apex_prev_y;
    uint8_t width[256], learned[256], last_mid[256], path_valid[256];
} track_nav_t;

track_element_t track_visual_classify(ring_vision_t *, const uint8_t *,
    const uint8_t *, const uint8_t *, uint8_t, uint8_t);
void abc_element_update(track_detector_t *, track_element_t);
// left/right/mid输出为控制用补线；识别必须先使用原始边线。
void track_nav_update(track_nav_t *nav, const track_detector_t *detector,
                      const uint8_t *raw_left, const uint8_t *raw_right, const uint8_t *binary,
                      uint8_t *left, uint8_t *right, uint8_t *mid,
                      uint8_t rows, uint8_t cols, uint32_t now_ms);
const char *track_nav_name(track_nav_phase_t phase);

void get_hist_gram(unsigned char* _img,
				   const unsigned short _rows, 
				   const unsigned short _cols, 
				   short* _hist_gram);
unsigned char get_threshold_otsu(const short* _hist_gram);
void binaryzation_process(unsigned char* _img, 
						  const unsigned short _rows, 
						  const unsigned short _cols, 
						  const unsigned int _threshold_value);
void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols, 
					   unsigned char _threshold_val, 
					   uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line);


#endif
