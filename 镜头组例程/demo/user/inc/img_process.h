#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#include "zf_common_headfile.h"

// 基于边线开口的初步识别，阈值需用实际赛道图像标定。
#define ELEMENT_CONFIRM_FRAMES  (3)
#define ELEMENT_RELEASE_FRAMES  (5)
#define ELEMENT_GAP_MIN_DIV     (20) // 连续开口至少图像高度的1/20
#define ELEMENT_GAP_MAX_DIV     (3)  // 排除大范围丢线
#define ELEMENT_EDGE_STEP_DIV   (20) // 环岛外侧边线允许的相邻行变化
#define ROUND_EDGE_MARGIN       (2)  // 距图像边缘2像素以内也按开口检查
#define ROUND_GAP_MIN_DIV       (30) // 环岛开口最少H/30行，120行图像为4行
#define ROUND_GAP_MAX_PERCENT   (45) // 最多占图像高度45%，仍排除整侧丢线
#define ROUND_WIDTH_PERCENT     (120) // 比上下参考赛道宽度至少增加20%
#define ROUND_ANCHOR_WINDOW     (3)
#define ROUND_ANCHOR_REQUIRED   (2)  // 上下各3行中至少2行恢复双边线
#define ROUND_CORNER_MIN_DIV    (12) // 开口侧端点必须明显离开画面边缘，排除缓慢出画弯道
#define ROUND_OUTER_VISIBLE_PERCENT (70) // 参考边线检查区至少70%的行实际可见
#define ROUND_OUTER_LOSS_MAX_DIV (20) // 每段出画最多H/20行，120行图像为6行
#define ROUND_OUTER_CLIP_MARGIN_DIV (12) // 缺口两端距对应画面边缘不超过W/12
#define ROUND_CONFIRM_FRAMES     (5) // 环岛连续5帧同向确认；十字仍为3帧

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
} track_detector_t;

track_element_t track_element_classify(const uint8_t *left, const uint8_t *right,
                                      uint8_t rows, uint8_t cols);
void track_element_update(track_detector_t *state, track_element_t candidate);
const char *track_element_name(track_element_t element);

// 视觉状态机初始参数，需按实际环岛尺寸、帧率和相机安装标定。
#define NAV_FEATURE_FRAMES       (3)
#define NAV_EXIT_STRAIGHT_FRAMES (5)
#define NAV_STAGE_MIN_MS         (150u)
#define NAV_INSIDE_MIN_MS        (500u)
#define NAV_HOLD_MS              (200u)
#define NAV_TIMEOUT_MS           (8000u)
#define NAV_RECOVER_MS           (600u)
#define NAV_RECOVER_TIMEOUT_MS   (3000u)
#define NAV_FAR_SUPPORT_ROWS    (6) // 连续真实双边线用于远端道路方向
#define NAV_FAR_MAX_SLOPE        (2) // 每行中线最大变化像素
#define NAV_ENTRY_BIAS_PERCENT   (18) // 入环时向开口侧偏移参考赛道宽度的百分比

typedef enum
{
    NAV_NORMAL = 0, NAV_CROSS, NAV_APPROACH, NAV_ENTER,
    NAV_INSIDE, NAV_EXIT, NAV_RECOVER, NAV_SLOW
} track_nav_phase_t;

typedef struct
{
    track_nav_phase_t phase;
    track_element_t element;
    uint32_t started_ms, stage_ms, last_path_ms;
    uint8_t feature_frames, straight_frames;
    uint8_t exit_anchor, guide_valid, drive_valid;
    uint8_t width[256], learned[256], last_mid[256], path_valid[256];
} track_nav_t;

// left/right/mid输出为控制用补线；识别必须先使用原始边线。
void track_nav_update(track_nav_t *nav, const track_detector_t *detector,
                      const uint8_t *raw_left, const uint8_t *raw_right,
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
