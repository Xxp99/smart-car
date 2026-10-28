/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */


#include "img_process.h"
#include "string.h"

/*
 *@func: 二值化函数
 */
void binaryzation_process(unsigned char* _img, 
                          const unsigned short _rows, 
                          const unsigned short _cols, 
                          const unsigned int _threshold_value)
{
	unsigned short _size = _rows * _cols;
	for (unsigned short _i = 0; _i < _size; ++_i)
	{
		if (*(_img + _i) > _threshold_value)
			*(_img + _i) = 255;
		else
			*(_img + _i) = 0;
	}
}

/*
 *@func: 获取直方图
 */
void get_hist_gram(unsigned char* _img,
				   const unsigned short _rows, 
				   const unsigned short _cols, 
				   short* _hist_gram)
{
	memset(_hist_gram, 0, 256 * sizeof(*_hist_gram));
    for (unsigned short _i = 0; _i < _rows; ++_i)
    {
        for (unsigned short _j = 0; _j < _cols; ++_j)
        {
            _hist_gram[_img[_i * _cols + _j]]++;
        }
    }
}

/*
 *@func: 大津法求阈值
 */
unsigned char get_threshold_otsu(const short* _hist_gram)
{
    int X, Y, amount = 0;
    int pixel_back = 0, pixel_fore = 0, pixel_integral_back = 0, pixel_integral_fore = 0, pixel_integral = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int pixel_min_value, pixel_max_value;
    int threshold = 0;

    for (pixel_min_value = 0; pixel_min_value < 256 && _hist_gram[pixel_min_value] == 0; pixel_min_value++)
        ;
    for (pixel_max_value = 255; pixel_max_value > pixel_min_value && _hist_gram[pixel_max_value] == 0; pixel_max_value--)
        ;
    if (pixel_max_value == pixel_min_value)
        return pixel_max_value; // 图像中只有一个颜色
    if (pixel_min_value + 1 == pixel_max_value)
        return pixel_min_value; // 图像中只有二个颜色

    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        amount += _hist_gram[Y]; //  像素总数

    pixel_integral = 0;
    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        pixel_integral += _hist_gram[Y] * Y;
    SigmaB = -1;
    for (Y = pixel_min_value; Y < pixel_max_value; Y++)
    {
        pixel_back = pixel_back + _hist_gram[Y];
        pixel_fore = amount - pixel_back;
        OmegaBack = (double)pixel_back / amount;
        OmegaFore = (double)pixel_fore / amount;
        pixel_integral_back += _hist_gram[Y] * Y;
        pixel_integral_fore = pixel_integral - pixel_integral_back;
        MicroBack = (double)pixel_integral_back / pixel_back;
        MicroFore = (double)pixel_integral_fore / pixel_fore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            threshold = Y;
        }
    }
    return threshold;
}

void auxiliary_process(uint8_t* _src_pixel_mat, uint8_t _src_rows, uint8_t _src_cols, 
					   unsigned char _threshold_val, 
					   uint8_t* _left_line, uint8_t* _mid_line, uint8_t* _right_line)
{
    // 输入已经是0/255二值图，不能继续用可能为0的大津阈值比较。
    _threshold_val = 127;
    uint8_t mid_point = _src_cols >> 1;
    if (_src_pixel_mat[(_src_rows - 1) * _src_cols + mid_point] < _threshold_val)
    {
        // 重新寻找扫线中点
        int16_t _tmp_left_mid = 0;
        int16_t _tmp_right_mid = _src_cols - 1;
        for (int16_t j = (_src_cols >> 1); j < _src_cols - 2; ++j)
        {
            if (_src_pixel_mat[(_src_rows - 1) * _src_cols + j] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j + 1] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j + 2] > _threshold_val)
            {
                _tmp_right_mid = j + 2;
                break;
            }
        }

        for (int16_t j = (_src_cols >> 1); j > 2; --j)
        {
            if (_src_pixel_mat[(_src_rows - 1) * _src_cols + j] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j - 1] > _threshold_val && 
				_src_pixel_mat[(_src_rows - 1) * _src_cols + j - 2] > _threshold_val)
            {
                _tmp_left_mid = j - 2;
                break;
            }
        }
        if (_tmp_right_mid - (_src_cols >> 1) < (_src_cols >> 1) - _tmp_left_mid)
        {
            mid_point = _tmp_right_mid;
        }
        else
        {
            mid_point = _tmp_left_mid;
        }
    }
    for (int i = _src_rows - 1; i >= 0; --i)
    {
        uint8_t cur_point = mid_point;
        // 即使扫描循环未执行，也清掉上一帧的边线位置。
        _left_line[i] = 0;
        _right_line[i] = _src_cols - 1;
       // 扫描左线
        while (cur_point - 2 > 0)
        {
            _left_line[i] = 0;
            if (_src_pixel_mat[i * _src_cols + cur_point] < _threshold_val && 
				_src_pixel_mat[i * _src_cols + cur_point - 1] < _threshold_val && 
				_src_pixel_mat[i * _src_cols + cur_point - 2] < _threshold_val)
            {
                _left_line[i] = cur_point;
                break;
            }
            --cur_point;
        }
        // 扫描右线
        cur_point = mid_point;
        while (cur_point + 2 < _src_cols)
        {
            _right_line[i] = _src_cols - 1;
            if (_src_pixel_mat[i * _src_cols + cur_point] < _threshold_val &&
				_src_pixel_mat[i * _src_cols + cur_point + 1] < _threshold_val &&
				_src_pixel_mat[i * _src_cols + cur_point + 2] < _threshold_val)
            {
                _right_line[i] = cur_point;
                break;
            }
            ++cur_point;
        }

        _mid_line[i] = (_right_line[i] + _left_line[i]) >> 1;
        if (_left_line[i] != 0 || 
			_right_line[i] != _src_cols - 1)
        {
            mid_point = _mid_line[i];
        }
        else
        {
            mid_point = _src_cols >> 1;
        }
    }
}

// 0=双边有效，1=左开口，2=右开口，3=双开口，4=无效几何。
static uint8_t track_row_kind(uint8_t left, uint8_t right, uint8_t cols)
{
    if (left >= right || right >= cols)
    {
        return 4;
    }
    return (left == 0 ? 1 : 0) | (right == cols - 1 ? 2 : 0);
}

// 环岛专用边缘容差，不改变十字判据或实际提取的边线。
static uint8_t round_row_kind(uint8_t left, uint8_t right, uint8_t cols)
{
    if (left >= right || right >= cols) return 4;
    return (left <= ROUND_EDGE_MARGIN ? 1 : 0) |
           (right >= cols - 1 - ROUND_EDGE_MARGIN ? 2 : 0);
}

static int round_outer_visible(int x, int cols)
{
    return x > ROUND_EDGE_MARGIN && x < cols - 1 - ROUND_EDGE_MARGIN;
}

static int round_outer_lost(int x, int cols, int left_ring)
{
    // 仅允许从参考边线自己的一侧出画；另一侧边缘及非法坐标不能补。
    return left_ring ? x >= cols - 1 - ROUND_EDGE_MARGIN && x < cols :
                       x >= 0 && x <= ROUND_EDGE_MARGIN;
}

// [first,end)必须是真实丢线，两端各保留两个可见点来检查出画/回画趋势。
static int round_clip_supported(const uint8_t *outer, int first, int end,
                                int rows, int cols, int left_ring)
{
    if (first < 2 || end + 1 >= rows || end - first > rows / ROUND_OUTER_LOSS_MAX_DIV)
        return 0;
    int a = outer[first - 2], b = outer[first - 1];
    int c = outer[end], d = outer[end + 1];
    if (!round_outer_visible(a, cols) || !round_outer_visible(b, cols) ||
        !round_outer_visible(c, cols) || !round_outer_visible(d, cols)) return 0;
    // 转成到对应画面边缘的距离，左右环岛使用同一组判据。
    if (left_ring) { a = cols - 1 - a; b = cols - 1 - b;
                    c = cols - 1 - c; d = cols - 1 - d; }
    int step = cols / ELEMENT_EDGE_STEP_DIV;
    return b <= cols / ROUND_OUTER_CLIP_MARGIN_DIV &&
           c <= cols / ROUND_OUTER_CLIP_MARGIN_DIV &&
           b - a <= ROUND_EDGE_MARGIN && c - d <= ROUND_EDGE_MARGIN &&
           a - b <= step && d - c <= step;
}

static int round_outer_continuous(const uint8_t *outer, int first, int last,
                                  int rows, int cols, int left_ring)
{
    int visible = 0, previous = -1;
    for (int y = first; y <= last; )
    {
        if (round_outer_visible(outer[y], cols))
        {
            if (previous >= 0)
            {
                int delta = (int)outer[y] - outer[previous];
                int limit = (y - previous) * (cols / ELEMENT_EDGE_STEP_DIV);
                if (delta > limit || delta < -limit) return 0;
            }
            previous = y++;
            ++visible;
        }
        else
        {
            int begin = y;
            while (y <= last && round_outer_lost(outer[y], cols, left_ring)) ++y;
            if (y == begin || !round_clip_supported(outer, begin, y, rows, cols, left_ring))
                return 0;
        }
    }
    return visible * 100 >= (last - first + 1) * ROUND_OUTER_VISIBLE_PERCENT;
}

// 仅供控制补线：短缺口用本帧上下真实点插值，绝不写回识别用的原始边线。
static int round_outer_estimate(const uint8_t *outer, int row, int rows,
                               int cols, int left_ring, int *estimate)
{
    if (!round_outer_lost(outer[row], cols, left_ring)) return 0;
    int first = row, end = row + 1;
    while (first > 0 && round_outer_lost(outer[first - 1], cols, left_ring)) --first;
    while (end < rows && round_outer_lost(outer[end], cols, left_ring)) ++end;
    if (!round_clip_supported(outer, first, end, rows, cols, left_ring)) return 0;
    *estimate = outer[first - 1] + ((int)outer[end] - outer[first - 1]) *
                (row - first + 1) / (end - first + 1);
    return 1;
}

static track_element_t round_classify(const uint8_t *left, const uint8_t *right,
                                     uint8_t rows, uint8_t cols, int *saw_loss)
{
    track_element_t result = TRACK_NONE;
    // 边线恢复允许出现在原ROI外，但仍必须在真实图像中找到上下锚点。
    int start = rows / 8, end = rows - ROUND_ANCHOR_WINDOW;
    // 分别沿内侧开口扫描，外侧短暂出画不再把同一个开口切成两段。
    for (int side = 1; side <= 2; ++side)
    for (int y = start; y < end; )
    {
        int kind = round_row_kind(left[y], right[y], cols);
        if (kind == 4 || !(kind & side)) { ++y; continue; }
        int first = y;
        while (y < end)
        {
            kind = round_row_kind(left[y], right[y], cols);
            if (kind == 4 || !(kind & side)) break;
            ++y;
        }
        kind = side;
        int length = y - first;
        // 仍只关注原ROI相交的开口，避免顶部极远处的小特征触发。
        if (y <= rows / 4 || first >= rows * 9 / 10) continue;
        *saw_loss = 1;
        if (length < rows / ROUND_GAP_MIN_DIV || length * 100 > rows * ROUND_GAP_MAX_PERCENT ||
            first < start + ROUND_ANCHOR_WINDOW || y >= end) continue;

        int top_count = 0, bottom_count = 0, width_sum = 0;
        int top_anchor = -1, bottom_anchor = -1;
        for (int k = 0; k < ROUND_ANCHOR_WINDOW; ++k)
        {
            int top = first - 1 - k, bottom = y + k;
            if (round_row_kind(left[top], right[top], cols) == 0)
            {
                ++top_count; width_sum += right[top] - left[top];
                if (top_anchor < 0) top_anchor = top;
            }
            if (round_row_kind(left[bottom], right[bottom], cols) == 0)
            {
                ++bottom_count; width_sum += right[bottom] - left[bottom];
                if (bottom_anchor < 0) bottom_anchor = bottom;
            }
        }
        int anchors = top_count + bottom_count;
        if (top_count < ROUND_ANCHOR_REQUIRED || bottom_count < ROUND_ANCHOR_REQUIRED ||
            width_sum < anchors * (cols / 10)) continue;

        // 普通弯道缓慢滑出画面时，开口前后的边线会贴近边缘，没有明显开口角点。
        int top_jump = kind == 1 ? left[top_anchor] : cols - 1 - right[top_anchor];
        int bottom_jump = kind == 1 ? left[bottom_anchor] : cols - 1 - right[bottom_anchor];
        if (top_jump < cols / ROUND_CORNER_MIN_DIV || bottom_jump < cols / ROUND_CORNER_MIN_DIV)
            continue;

        const uint8_t *outer = kind == 1 ? right : left;
        if (!round_outer_continuous(outer, top_anchor, bottom_anchor, rows, cols, kind == 1))
            continue;
        int gap_width = 0, observed = 0;
        for (int k = first; k < y; ++k)
            if (round_outer_visible(outer[k], cols))
            {
                gap_width += right[k] - left[k];
                ++observed;
            }
        // 双侧丢线行不参与宽度证据，不能利用画面边界或补线虚增开口宽度。
        if (!observed || gap_width * anchors * 100 < width_sum * observed * ROUND_WIDTH_PERCENT)
            continue;
        track_element_t found = kind == 1 ? TRACK_ROUND_LEFT : TRACK_ROUND_RIGHT;
        if (result != TRACK_NONE && result != found) return TRACK_UNKNOWN;
        result = found;
    }
    return result;
}

track_element_t track_element_classify(const uint8_t *left, const uint8_t *right,
                                      uint8_t rows, uint8_t cols)
{
    int start = rows / 4;
    int end = rows * 9 / 10;
    int min_gap = rows / ELEMENT_GAP_MIN_DIV;
    int saw_loss = 0;
    track_element_t result = TRACK_NONE;
    if (!left || !right || rows < 40 || cols < 40)
    {
        return TRACK_UNKNOWN;
    }
    for (int y = start; y < end; )
    {
        uint8_t kind = track_row_kind(left[y], right[y], cols);
        if (kind == 0)
        {
            ++y;
            continue;
        }
        saw_loss = 1;
        int first = y;
        while (y < end && track_row_kind(left[y], right[y], cols) == kind)
        {
            ++y;
        }
        int length = y - first;
        // 单侧开口由下面独立的环岛判据处理，十字保持原来的严格条件。
        if (kind != 3) continue;
        // 开口必须在上下都有连续双边线，不把整片白图或出画弯道认成路口。
        if (kind == 4 || length < min_gap || length > rows / ELEMENT_GAP_MAX_DIV ||
            first < start + 3 || y + 2 >= end)
        {
            continue;
        }
        int anchors_ok = 1;
        int width_sum = 0;
        for (int k = 0; k < 3; ++k)
        {
            int top = first - 1 - k;
            int bottom = y + k;
            if (track_row_kind(left[top], right[top], cols) != 0 ||
                track_row_kind(left[bottom], right[bottom], cols) != 0)
            {
                anchors_ok = 0;
            }
            width_sum += (int)right[top] - left[top] + (int)right[bottom] - left[bottom];
        }
        if (!anchors_ok || width_sum < 6 * (cols / 10))
        {
            continue;
        }
        int gap_width = 0;
        for (int k = first; k < y; ++k)
        {
            gap_width += (int)right[k] - left[k];
        }
        // 十字开口宽度至少参考宽度的1.5倍。
        if (gap_width * 600 < width_sum * length * 150)
        {
            continue;
        }
        track_element_t found = TRACK_CROSS;
        if (result != TRACK_NONE && result != found)
        {
            return TRACK_UNKNOWN; // 同一帧存在冲突特征
        }
        result = found;
    }
    track_element_t round_result = round_classify(left, right, rows, cols, &saw_loss);
    if (round_result == TRACK_UNKNOWN ||
        (round_result != TRACK_NONE && result != TRACK_NONE)) return TRACK_UNKNOWN;
    if (round_result != TRACK_NONE) return round_result;
    return result != TRACK_NONE ? result : (saw_loss ? TRACK_UNKNOWN : TRACK_NONE);
}

void track_element_update(track_detector_t *state, track_element_t candidate)
{
    int required = candidate == TRACK_ROUND_LEFT || candidate == TRACK_ROUND_RIGHT ?
                   ROUND_CONFIRM_FRAMES : ELEMENT_CONFIRM_FRAMES;
    if (candidate == state->candidate)
    {
        if (state->confirm_count < required)
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
    if (candidate != TRACK_NONE && candidate != TRACK_UNKNOWN &&
        state->confirm_count >= required)
    {
        state->confirmed = candidate;
        state->release_count = 0;
    }
}

const char *track_element_name(track_element_t element)
{
    switch (element)
    {
        case TRACK_CROSS:       return "CROSS";
        case TRACK_ROUND_LEFT:  return "RND-L";
        case TRACK_ROUND_RIGHT: return "RND-R";
        case TRACK_UNKNOWN:    return "UNKWN";
        default:               return "NONE ";
    }
}

static int nav_clip(int value, int low, int high)
{
    return value < low ? low : (value > high ? high : value);
}

static int nav_pair(uint8_t left, uint8_t right, uint8_t cols)
{
    return left > 0 && right < cols - 1 && right - (int)left >= cols / 10;
}

static void nav_stage(track_nav_t *nav, track_nav_phase_t phase, uint32_t now)
{
    nav->phase = phase;
    nav->stage_ms = now;
    nav->feature_frames = 0;
    nav->straight_frames = 0;
}

// 十字：只连接上下真实边线之间的缺口，不把整片丢线当成可用路径。
static void nav_bridge(uint8_t *edge, int rows, int cols, int is_left)
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
            edge[k] = (uint8_t)nav_clip(edge[top] +
                ((int)edge[y] - edge[top]) * (k - top) / (y - top), 1, cols - 2);
        }
    }
}

// 远端道路必须有连续真实双边线，宽度与入元素前标定一致。
// 返回最靠近指定上界的有效段；不把插值或历史中线当作出口证据。
static int nav_far_road(const track_nav_t *nav, const uint8_t *left,
                        const uint8_t *right, int rows, int cols, int limit,
                        int *row, int *center, int *slope)
{
    for (int bottom = limit; bottom >= rows / 4 + NAV_FAR_SUPPORT_ROWS; --bottom)
    {
        int top = bottom - NAV_FAR_SUPPORT_ROWS + 1, ok = 1;
        for (int y = top; y <= bottom; ++y)
        {
            int width = right[y] - (int)left[y];
            if (!nav_pair(left[y], right[y], cols) || !nav->learned[y] ||
                width * 100 < nav->width[y] * 75 || width * 100 > nav->width[y] * 125)
            { ok = 0; break; }
            if (y > top && (((int)left[y] - left[y-1] > cols / ELEMENT_EDGE_STEP_DIV || (int)left[y-1] - left[y] > cols / ELEMENT_EDGE_STEP_DIV) ||
                            ((int)right[y] - right[y-1] > cols / ELEMENT_EDGE_STEP_DIV || (int)right[y-1] - right[y] > cols / ELEMENT_EDGE_STEP_DIV)))
            { ok = 0; break; }
        }
        if (!ok) continue;
        int c0 = (left[top] + right[top]) / 2;
        int c1 = (left[bottom] + right[bottom]) / 2;
        int delta = c1 - c0;
        if (delta > NAV_FAR_MAX_SLOPE * (bottom-top) ||
            delta < -NAV_FAR_MAX_SLOPE * (bottom-top)) continue;
        *row = bottom; *center = c1;
        *slope = delta * 256 / (bottom-top);
        return 1;
    }
    return 0;
}

// 出口要求外侧有明确开口端点，且开口上方存在真实出口道路。
static int nav_exit_road(const track_nav_t *nav, const uint8_t *left,
                         const uint8_t *right, int rows, int cols, int is_left,
                         int *row, int *center, int *slope)
{
    const uint8_t *outer = is_left ? right : left;
    for (int y = rows / 3; y < rows * 9 / 10; ++y)
    {
        if (!round_outer_lost(outer[y], cols, is_left)) continue;
        int first = y;
        while (y < rows && round_outer_lost(outer[y], cols, is_left)) ++y;
        if (y-first < rows/20 || first < NAV_FAR_SUPPORT_ROWS || y >= rows) continue;
        int before = outer[first-1], after = outer[y];
        if (!round_outer_visible(before, cols) || !round_outer_visible(after, cols)) continue;
        int d0 = is_left ? cols-1-before : before;
        int d1 = is_left ? cols-1-after : after;
        if (d0 < cols/ROUND_CORNER_MIN_DIV || d1 < cols/ROUND_CORNER_MIN_DIV) continue;
        if (nav_far_road(nav,left,right,rows,cols,first-1,row,center,slope) &&
            *row == first-1) return 1;
    }
    return 0;
}

void track_nav_update(track_nav_t *nav, const track_detector_t *detector,
                      const uint8_t *raw_left, const uint8_t *raw_right,
                      uint8_t *left, uint8_t *right, uint8_t *mid,
                      uint8_t rows, uint8_t cols, uint32_t now_ms)
{
    int near = rows * 4 / 5, far = rows * 3 / 5;
    int left_open = 0, right_open = 0, both_open = 0;
    int full = nav_pair(raw_left[near], raw_right[near], cols) &&
               nav_pair(raw_left[far], raw_right[far], cols);
    int near_mid = ((int)raw_left[near] + raw_right[near]) / 2;
    int far_mid = ((int)raw_left[far] + raw_right[far]) / 2;
    int straight = full && nav_clip(near_mid - far_mid, -cols / 16, cols / 16) ==
                                     near_mid - far_mid;
    for (int y = 0; y < rows; ++y)
    {
        left[y] = raw_left[y]; right[y] = raw_right[y];
        nav->path_valid[y] = nav_pair(left[y], right[y], cols);
        mid[y] = (uint8_t)(((int)left[y] + right[y]) / 2);
        // 只在普通赛道学习逐行宽度，进入特殊元素后冻结，防止开口污染。
        if ((nav->phase == NAV_NORMAL || nav->phase == NAV_SLOW) && detector->candidate == TRACK_NONE &&
            nav->path_valid[y] && right[y] - left[y] < cols * 4 / 5)
        {
            int width = right[y] - left[y];
            nav->width[y] = nav->learned[y] ? (3 * nav->width[y] + width + 2) / 4 : width;
            nav->learned[y] = 1;
        }
        if (y >= rows * 2 / 3 && y < rows * 9 / 10)
        {
            left_open += left[y] <= ROUND_EDGE_MARGIN && right[y] > ROUND_EDGE_MARGIN &&
                         right[y] < cols - 1 - ROUND_EDGE_MARGIN;
            right_open += right[y] >= cols - 1 - ROUND_EDGE_MARGIN && left[y] > ROUND_EDGE_MARGIN &&
                          left[y] < cols - 1 - ROUND_EDGE_MARGIN;
            both_open += left[y] == 0 && right[y] == cols - 1;
        }
    }
    if (nav->phase == NAV_NORMAL && detector->confirmed != TRACK_NONE &&
        detector->confirmed != TRACK_UNKNOWN)
    {
        nav->element = detector->confirmed;
        nav->started_ms = now_ms;
        nav_stage(nav, !nav->learned[near] || !nav->learned[far] ? NAV_SLOW :
            (nav->element == TRACK_CROSS ? NAV_CROSS : NAV_APPROACH), now_ms);
    }
    if (nav->phase != NAV_NORMAL && nav->phase != NAV_RECOVER && nav->phase != NAV_SLOW &&
        now_ms - nav->started_ms >= NAV_TIMEOUT_MS)
    {
        nav_stage(nav, NAV_SLOW, now_ms);
    }
    if (nav->phase == NAV_RECOVER && now_ms - nav->stage_ms >= NAV_RECOVER_TIMEOUT_MS)
    {
        nav_stage(nav, NAV_SLOW, now_ms);
    }
    int is_left = nav->element == TRACK_ROUND_LEFT;
    int inner_open = is_left ? left_open : right_open;
    int outer_open = is_left ? right_open : left_open;
    int guide_row = 0, guide_center = 0, guide_slope = 0;
    int exit_evidence = nav_exit_road(nav, raw_left, raw_right, rows, cols,
                                     is_left, &guide_row, &guide_center, &guide_slope);
    nav->guide_valid = 0;
    int feature = 0;
    if (nav->phase == NAV_APPROACH) feature = inner_open >= rows / 20;
    if (nav->phase == NAV_ENTER) feature = full && inner_open == 0;
    if (nav->phase == NAV_INSIDE) feature = outer_open >= rows / 20 && both_open == 0 && exit_evidence;
    if (feature) { if (nav->feature_frames < NAV_FEATURE_FRAMES) ++nav->feature_frames; }
    else nav->feature_frames = 0;
    if ((nav->phase == NAV_SLOW ? full : straight) && detector->candidate == TRACK_NONE)
    {
        if (nav->straight_frames < NAV_EXIT_STRAIGHT_FRAMES) ++nav->straight_frames;
    }
    else nav->straight_frames = 0;

    uint32_t elapsed = now_ms - nav->stage_ms;
    if (nav->phase == NAV_APPROACH && nav->feature_frames >= NAV_FEATURE_FRAMES)
        nav_stage(nav, NAV_ENTER, now_ms);
    else if (nav->phase == NAV_ENTER && elapsed >= NAV_STAGE_MIN_MS &&
             nav->feature_frames >= NAV_FEATURE_FRAMES)
        nav_stage(nav, NAV_INSIDE, now_ms);
    else if (nav->phase == NAV_INSIDE && elapsed >= NAV_INSIDE_MIN_MS &&
             nav->feature_frames >= NAV_FEATURE_FRAMES)
    {
        nav->exit_anchor = nav->last_mid[rows - 1];
        nav_stage(nav, NAV_EXIT, now_ms);
    }
    else if ((nav->phase == NAV_EXIT || nav->phase == NAV_CROSS) &&
             elapsed >= NAV_STAGE_MIN_MS && nav->straight_frames >= NAV_EXIT_STRAIGHT_FRAMES)
        nav_stage(nav, NAV_RECOVER, now_ms);
    else if ((nav->phase == NAV_RECOVER || nav->phase == NAV_SLOW) && elapsed >= NAV_RECOVER_MS &&
             nav->straight_frames >= NAV_EXIT_STRAIGHT_FRAMES && detector->confirmed == TRACK_NONE)
    {
        nav_stage(nav, NAV_NORMAL, now_ms);
        nav->element = TRACK_NONE;
    }

    if (nav->phase == NAV_EXIT)
    {
        // 开口消失后允许继续使用出口双边道路；不退回沿岛边模式。
        nav->guide_valid = exit_evidence || nav_far_road(nav, raw_left, raw_right,
            rows, cols, far, &guide_row, &guide_center, &guide_slope);
    }
    if (nav->phase == NAV_CROSS)
    {
        nav->guide_valid = nav_far_road(nav, raw_left, raw_right, rows, cols,
            far, &guide_row, &guide_center, &guide_slope);
        nav_bridge(left, rows, cols, 1);
        nav_bridge(right, rows, cols, 0);
    }
    for (int y = rows / 4; y < rows; ++y)
    {
        int usable = nav_pair(left[y], right[y], cols);
        int center = ((int)left[y] + right[y]) / 2;
        int width = nav->width[y];
        int ring = nav->phase == NAV_APPROACH || nav->phase == NAV_ENTER ||
                   nav->phase == NAV_INSIDE || nav->phase == NAV_EXIT;
        if (ring && nav->phase != NAV_EXIT && nav->learned[y])
        {
            int inner_valid = is_left ? raw_left[y] > ROUND_EDGE_MARGIN && raw_left[y] < cols - 1 - ROUND_EDGE_MARGIN :
                                       raw_right[y] > ROUND_EDGE_MARGIN && raw_right[y] < cols - 1 - ROUND_EDGE_MARGIN;
            int outer_valid = is_left ? raw_right[y] > ROUND_EDGE_MARGIN && raw_right[y] < cols - 1 - ROUND_EDGE_MARGIN :
                                       raw_left[y] > ROUND_EDGE_MARGIN && raw_left[y] < cols - 1 - ROUND_EDGE_MARGIN;
            int outer_position = is_left ? raw_right[y] : raw_left[y];
            if (raw_left[y] >= raw_right[y]) inner_valid = outer_valid = 0;
            else if (!outer_valid)
                outer_valid = round_outer_estimate(is_left ? raw_right : raw_left,
                                                   y, rows, cols, is_left, &outer_position);
            usable = 0;
            // 入口沿外侧边线，入环偏向开口；环内沿岛边，出口另建连接线。
            if (nav->phase == NAV_INSIDE && inner_valid)
            {
                center = is_left ? raw_left[y] + width / 2 : raw_right[y] - width / 2;
                usable = 1;
            }
            else if (outer_valid)
            {
                center = is_left ? outer_position - width / 2 : outer_position + width / 2;
                usable = 1;
            }
            if (usable && nav->phase == NAV_ENTER)
                center += (is_left ? -1 : 1) * width * NAV_ENTRY_BIAS_PERCENT / 100;
            if (usable)
            {
                left[y] = (uint8_t)nav_clip(center - width / 2, 1, cols - 2);
                right[y] = (uint8_t)nav_clip(center + width / 2, 1, cols - 2);
                usable = nav_pair(left[y], right[y], cols);
            }
        }
        if (nav->phase == NAV_EXIT)
        {
            usable = 0;
            if (nav->guide_valid && nav->learned[y] && y >= guide_row)
            {
                // 当前出口道路连接至入EXIT时车前路径，避免继续绕岛。
                center = guide_center + ((int)nav->exit_anchor - guide_center) *
                         (y-guide_row) / (rows-1-guide_row);
                usable = center-width/2 > 0 && center+width/2 < cols-1;
                if (usable) { left[y] = (uint8_t)(center-width/2);
                              right[y] = (uint8_t)(center+width/2); }
            }
            else if (y < guide_row) usable = nav_pair(raw_left[y],raw_right[y],cols);
        }
        if (nav->phase == NAV_CROSS && !usable && nav->guide_valid &&
            nav->learned[y] && y >= guide_row && y-guide_row <= rows/2)
        {
            // 近端锚点消失时，只按连续远端道路方向做有界延伸。
            center = guide_center + guide_slope * (y-guide_row) / 256;
            if (center-width/2 > 0 && center+width/2 < cols-1)
            {
                left[y] = (uint8_t)(center-width/2);
                right[y] = (uint8_t)(center+width/2);
                usable = 1;
            }
        }
        // 十字/退出恢复时的单边丢线，用进入前学习的该行宽度补另一侧。
        if (!usable && nav->learned[y] &&
            (nav->phase == NAV_CROSS || nav->phase == NAV_RECOVER || nav->phase == NAV_SLOW))
        {
            if (left[y] == 0 && right[y] > 0 && right[y] < cols - 1)
                left[y] = (uint8_t)nav_clip(right[y] - width, 1, cols - 2);
            else if (right[y] == cols - 1 && left[y] > 0 && left[y] < cols - 1)
                right[y] = (uint8_t)nav_clip(left[y] + width, 1, cols - 2);
            usable = nav_pair(left[y], right[y], cols);
            center = ((int)left[y] + right[y]) / 2;
        }
        // 真实/补线可用性与短时保持分开，保持不能刷新last_path_ms。
        nav->path_valid[y] = usable;
        if (usable)
        {
            if (!ring) center = ((int)left[y] + right[y]) / 2;
            mid[y] = (uint8_t)nav_clip(center, 1, cols - 2);
            nav->last_mid[y] = mid[y];
        }
        else if (nav->phase != NAV_NORMAL && nav->learned[y] &&
                 now_ms - nav->last_path_ms <= NAV_HOLD_MS)
            mid[y] = nav->last_mid[y];
    }
    if (nav->path_valid[near] && nav->path_valid[far]) nav->last_path_ms = now_ms;
    nav->drive_valid = (nav->path_valid[near] && nav->path_valid[far]) ||
        (nav->phase != NAV_NORMAL && now_ms-nav->last_path_ms <= NAV_HOLD_MS);
    if (nav->phase != NAV_NORMAL && nav->phase != NAV_SLOW &&
             now_ms - nav->last_path_ms > NAV_HOLD_MS)
    {
        nav_stage(nav, NAV_SLOW, now_ms);
        // 丢线退出特殊动作的当帧立即撤掉环岛偏移，回到原始中线。
        for (int y = 0; y < rows; ++y)
        {
            left[y] = raw_left[y]; right[y] = raw_right[y];
            mid[y] = (uint8_t)(((int)raw_left[y] + raw_right[y]) / 2);
        }
    }
}

const char *track_nav_name(track_nav_phase_t phase)
{
    switch (phase)
    {
        case NAV_CROSS: return "CROSS";
        case NAV_APPROACH: return "APPR ";
        case NAV_ENTER: return "ENTER";
        case NAV_INSIDE: return "IN   ";
        case NAV_EXIT: return "EXIT ";
        case NAV_RECOVER: return "BACK ";
        case NAV_SLOW: return "SLOW ";
        default: return "RUN  ";
    }
}


