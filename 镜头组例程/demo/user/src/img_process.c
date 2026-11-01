/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */


#include "img_process.h"
#include "string.h"
#include <stdlib.h>

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
        // One continuous reference edge and two supported opening endpoints.
        // Do not compare sentinel-based opening width with ordinary road width.
        track_element_t found = kind == 1 ? TRACK_ROUND_LEFT : TRACK_ROUND_RIGHT;
        if (result != TRACK_NONE && result != found) return TRACK_UNKNOWN;
        result = found;
    }
    return result;
}

static track_element_t track_cross_classify(const uint8_t *left, const uint8_t *right,
                                            uint8_t rows, uint8_t cols)
{
    int start = rows / 8, end = rows - 5;
    int saw_loss = 0;
    track_element_t result = TRACK_NONE;
    if (!left || !right || rows < 40 || cols < 40) return TRACK_UNKNOWN;
    for (int y = start; y < end; )
    {
        int kind = round_row_kind(left[y], right[y], cols);
        if (kind == 0) { ++y; continue; }
        saw_loss = 1;
        if (kind == 4) { ++y; continue; }
        int first = y, overlap = 0;
        // Union of left/right openings permits a slanted cross with staggered corners.
        while (y < end)
        {
            kind = round_row_kind(left[y], right[y], cols);
            if (kind == 0 || kind == 4) break;
            if (kind == 3) ++overlap;
            ++y;
        }
        int length = y - first;
        if (y <= rows/4 || first >= rows*9/10 || first < start+5 || y >= end ||
            overlap < rows/30 || overlap*2 < length || length*100 > rows*45) continue;
        int top_count = 0, bottom_count = 0, width_sum = 0;
        for (int k = 0; k < 5; ++k)
        {
            int top = first-1-k, bottom = y+k;
            if (round_row_kind(left[top],right[top],cols)==0)
            { ++top_count; width_sum += right[top]-left[top]; }
            if (round_row_kind(left[bottom],right[bottom],cols)==0)
            { ++bottom_count; width_sum += right[bottom]-left[bottom]; }
        }
        int anchors = top_count+bottom_count;
        if (top_count<2 || bottom_count<2 || width_sum < anchors*(cols/10)) continue;
        // Missing-edge sentinels are not measured road width: do not use
        // their separation as a cross/normal-road expansion ratio.
        result = TRACK_CROSS;
    }
    return result != TRACK_NONE ? result : (saw_loss ? TRACK_UNKNOWN : TRACK_NONE);
}

track_element_t track_element_classify(const uint8_t *left, const uint8_t *right,
                                      uint8_t rows, uint8_t cols)
{
    int saw_loss = 0;
    if (!left || !right || rows < 40 || cols < 40) return TRACK_UNKNOWN;
    track_element_t cross_result = track_cross_classify(left, right, rows, cols);
    track_element_t round_result = round_classify(left, right, rows, cols, &saw_loss);
    if (round_result == TRACK_UNKNOWN ||
        (round_result != TRACK_NONE && cross_result == TRACK_CROSS)) return TRACK_UNKNOWN;
    if (round_result != TRACK_NONE) return round_result;
    if (cross_result == TRACK_CROSS) return TRACK_CROSS;
    return (cross_result == TRACK_UNKNOWN || saw_loss) ? TRACK_UNKNOWN : TRACK_NONE;
}

// A clipped opening has only one visible corner. Require a smooth opposite
// boundary, not just continuity; the other corner is never synthesized.
static int round_partial(const uint8_t *left,const uint8_t *right,int rows,int cols,
                          int is_left,int *corner,int *kind)
{
    const uint8_t *inner=is_left?left:right,*outer=is_left?right:left;
    int a=0,b=rows;
    if(round_outer_lost(inner[0],cols,!is_left)) {
        while(a<rows && round_outer_lost(inner[a],cols,!is_left)) ++a;
        b=a; a=0; *kind=1; *corner=b;
    } else if(round_outer_lost(inner[rows-1],cols,!is_left)) {
        a=rows-1;
        while(a>0 && round_outer_lost(inner[a-1],cols,!is_left)) --a;
        *kind=2; *corner=a-1;
    } else return 0;
    if(b-a<rows/ROUND_GAP_MIN_DIV || *corner<rows/4 || *corner>=rows-3) return 0;
    for(int k=0;k<3;++k) {
        int y=*corner+(*kind==1?k:-k);
        if(round_row_kind(left[y],right[y],cols)!=0 || right[y]-left[y]<cols/10 ||
           (is_left?inner[y]:cols-1-inner[y])<cols/ROUND_CORNER_MIN_DIV) return 0;
        if(k && abs((int)inner[y]-inner[*corner])>cols/20) return 0;
    }
    int first=rows/8,last=rows-1;
    if(!round_outer_continuous(outer,first,last,rows,cols,is_left)) return 0;
    while(first<last && !round_outer_visible(outer[first],cols)) ++first;
    while(last>first && !round_outer_visible(outer[last],cols)) --last;
    if(last-first<rows/2) return 0;
    for(int y=first;y<=last;++y) {
        if(left[y]>=right[y]) return 0;
        if(!round_outer_visible(outer[y],cols)) continue;
        int fit=outer[first]+((int)outer[last]-outer[first])*(y-first)/(last-first);
        if(abs((int)outer[y]-fit)>cols/32) return 0;
    }
    return 1;
}

void track_element_observe(track_detector_t *state,const uint8_t *left,const uint8_t *right,
                           uint8_t rows,uint8_t cols,uint32_t now_ms)
{
    track_element_t candidate=track_element_classify(left,right,rows,cols);
    int side=0,corner=0,kind=0,c=0,k=0;
    if((candidate==TRACK_UNKNOWN || candidate==TRACK_NONE) &&
       left && right && rows>=40 && cols>=40) {
        if(round_partial(left,right,rows,cols,1,&corner,&kind)) side=1;
        if(round_partial(left,right,rows,cols,0,&c,&k)) {
            if(side) side=0;
            else {side=2;corner=c;kind=k;}
        }
    }
    if(side) {
        if(state->sequence_side!=side ||
           now_ms-state->sequence_last_ms>=ROUND_SEQUENCE_GAP_MS ||
           now_ms-state->sequence_start_ms>=ROUND_SEQUENCE_MAX_MS) {
            state->sequence_side=(uint8_t)side;
            state->sequence_kind=(uint8_t)kind;
            state->sequence_row=(uint8_t)corner;
            state->sequence_progress=0;
            state->sequence_start_ms=now_ms;
            state->confirm_count=0;
        } else if((state->sequence_kind==kind && corner>=state->sequence_row+2 &&
                   corner<=state->sequence_row+rows/3) ||
                  (state->sequence_kind!=kind && abs(corner-state->sequence_row)<=rows/2))
            state->sequence_progress=1;
        state->sequence_last_ms=now_ms;
        if(state->sequence_progress) candidate=side==1?TRACK_ROUND_LEFT:TRACK_ROUND_RIGHT;
    } else {
        state->sequence_side=state->sequence_progress=0;
    }
    track_element_update(state,candidate);
}

// Robust local bands replace the two exact-row requirement for telemetry only.
uint8_t track_road_shape(const uint8_t *left, const uint8_t *right,
                         uint8_t rows, uint8_t cols)
{
    int centers[2];
    if (!left || !right || rows < 40 || cols < 40) return 0;
    for (int band=0; band<2; ++band)
    {
        int row = band ? rows*3/5 : rows*4/5;
        int count=0, sum=0, previous=-1;
        for (int y=row-4; y<=row+4; ++y)
        {
            int width = (int)right[y]-left[y];
            if (left[y]==0 || right[y]>=cols-1 || width<cols/10) continue;
            int center = (left[y]+right[y])/2;
            if (previous>=0 && (center-previous>8 || previous-center>8)) return 0;
            previous=center; sum+=center; ++count;
        }
        if (count<3) return 0;
        centers[band]=sum/count;
    }
    int offset=centers[0]-cols/2, change=centers[1]-centers[0];
    return offset>=-5 && offset<=5 && change>=-5 && change<=5 ? 1 : 2;
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
    track_nav_phase_t previous_phase = nav->phase;
    if (phase == NAV_SLOW &&
        (nav->phase == NAV_APPROACH || nav->phase == NAV_ENTER ||
         nav->phase == NAV_INSIDE || nav->phase == NAV_EXIT) &&
        (nav->element == TRACK_ROUND_LEFT || nav->element == TRACK_ROUND_RIGHT))
        nav->slow_resume_phase = nav->phase;
    else if (phase == NAV_RECOVER && nav->phase == NAV_EXIT &&
             (nav->element == TRACK_ROUND_LEFT || nav->element == TRACK_ROUND_RIGHT))
        nav->slow_resume_phase = NAV_EXIT;
    nav->phase = phase;
    if (phase==NAV_INSIDE || phase==NAV_EXIT || phase==NAV_NORMAL) nav->entry_pending=0;
    if (phase==NAV_NORMAL) {
        nav->exit_guide_valid=0;
        nav->slow_resume_phase=NAV_NORMAL;
    }
    nav->stage_ms = now;
    if (phase == NAV_EXIT &&
        !(previous_phase == NAV_SLOW && nav->slow_resume_phase == NAV_EXIT) &&
        (nav->element == TRACK_ROUND_LEFT || nav->element == TRACK_ROUND_RIGHT))
        nav->exit_turn_until_ms = now + NAV_RING_EXIT_TURN_MS;
    nav->feature_frames = 0;
    nav->straight_frames = 0;
    if (phase == NAV_INSIDE || phase == NAV_EXIT || phase == NAV_NORMAL)
        nav->exit_confirm_frames = 0;
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
        if (d0 < cols/ROUND_EXIT_CORNER_MIN_DIV || d1 < cols/ROUND_EXIT_CORNER_MIN_DIV) continue;
        if (nav_far_road(nav,left,right,rows,cols,first-1,row,center,slope) &&
            *row <= first-1 && first-1-*row <= rows/NAV_EXIT_ROW_TOLERANCE_DIV)
            return 1;
    }
    return 0;
}

// Detect an interior black cap on the island side. Reject edge-connected black
// background and isolated pixels; require white above and four supported rows.
static void nav_ring_apex(track_nav_t *nav, const uint8_t *binary,
                           int rows, int cols, int is_left, uint32_t now_ms)
{
    if ((uint32_t)(now_ms-nav->apex_ms)>=300u) nav->apex_hits=0;
    nav->apex_ms=now_ms;
    int found=0, ax=0, ay=0;
    nav->apex_valid=0;
    if (!binary || (nav->phase!=NAV_APPROACH && nav->phase!=NAV_ENTER))
    { nav->apex_hits=0; nav->apex_valid=0; return; }
    for (int y=rows/8; y<rows*3/5 && !found; ++y)
    for (int x=3; x<cols-3; )
    {
        if (binary[y*cols+x]!=0) { ++x; continue; }
        int first=x;
        while (x<cols-3 && binary[y*cols+x]==0) ++x;
        int last=x-1, center=(first+last)/2, width=x-first, ok=1;
        if (first<=3 || x>=cols-3 || width<3 || width>cols/3 ||
            (is_left ? center>=cols/2 : center<=cols/2)) continue;
        for (int k=first-1;k<=last+1;++k)
            if (binary[(y-1)*cols+k]==0 || binary[(y-2)*cols+k]==0) ok=0;
        int previous_width=width;
        for (int dy=1;dy<=3 && ok;++dy)
        {
            int l=center,r=center;
            if (binary[(y+dy)*cols+center]!=0) {ok=0;break;}
            while(l>1 && binary[(y+dy)*cols+l-1]==0) --l;
            while(r<cols-2 && binary[(y+dy)*cols+r+1]==0) ++r;
            int w=r-l+1;
            if(l<=1 || r>=cols-2 || w<previous_width ||
               w-previous_width>cols/8 || w>cols/2) ok=0;
            previous_width=w;
        }
        if(ok && previous_width>=width+2) {found=1;ax=center;ay=y;break;}
    }
    if (!found) {nav->apex_hits=0;nav->apex_valid=0;return;}
    int dx=ax-nav->apex_prev_x,dy=ay-nav->apex_prev_y;
    // 车速提高时，黑色岛尖会在相邻帧移动超过8像素；按图像尺寸容忍位移，
    // 仍要求连续两帧的岛尖位置接近，避免单帧噪点接管补线。
    if(nav->apex_hits && dx>=-cols/12 && dx<=cols/12 &&
       dy>=-rows/12 && dy<=rows/12)
    {if(nav->apex_hits<2) ++nav->apex_hits;}
    else {nav->apex_hits=1;nav->apex_valid=0;}
    nav->apex_prev_x=(uint8_t)ax; nav->apex_prev_y=(uint8_t)ay;
    if(nav->apex_hits>=2)
    {nav->apex_x=(uint8_t)ax;nav->apex_y=(uint8_t)ay;nav->apex_valid=1;}
}

// Follow the current cap's connected black component. The road-facing edge is
// a boundary, never a centerline target. Stop on loss/branch/jump; no extrapolation.
static void nav_ring_contour(const track_nav_t *nav, const uint8_t *binary,
                             int rows, int cols, int is_left, uint8_t *edge)
{
    memset(edge, 0, rows);
    if (!nav->apex_valid || !binary) return;
    int previous_left=nav->apex_x, previous_right=nav->apex_x;
    for (int y=nav->apex_y; y<rows; ++y)
    {
        // 沿岛边逐行跟踪时允许小幅横移，否则斜坡状岛尖一偏1像素就会断线。
        int reach=cols/20;
        int search_left=nav_clip(previous_left-reach,1,cols-2);
        int search_right=nav_clip(previous_right+reach,1,cols-2);
        int x=search_left, l, r;
        while (x<=search_right && binary[y*cols+x]!=0) ++x;
        if (x>search_right) break;
        l=r=x;
        while (l>1 && binary[y*cols+l-1]==0) --l;
        while (r<cols-2 && binary[y*cols+r+1]==0) ++r;
        // 搜索带内出现第二段黑色区域时视作分叉/噪声，不拿它补线。
        for (x=r+1; x<=search_right; ++x)
            if (binary[y*cols+x]==0) break;
        if (x<=search_right || l<=1 || r>=cols-2 || r-l+1>cols*3/4 ||
            (y>nav->apex_y &&
             (l-previous_left>cols/20 || previous_left-l>cols/20 ||
              r-previous_right>cols/20 || previous_right-r>cols/20))) break;
        edge[y]=(uint8_t)(is_left?r:l);
        previous_left=l; previous_right=r;
    }
}

// A near opening may guide ENTER without a visible cap. Require current real
// endpoints and the same bounded outer-edge support used by recognition.
static int nav_entry_gap(const uint8_t *left, const uint8_t *right,
                         int row, int rows, int cols, int is_left, int *first, int *end)
{
    const uint8_t *inner=is_left?left:right, *outer=is_left?right:left;
    int a=row,b=row;
    // round_outer_lost's direction is reversed for the inner edge.
    if (!round_outer_lost(inner[row],cols,!is_left)) return 0;
    while(a>0 && round_outer_lost(inner[a-1],cols,!is_left)) --a;
    while(b<rows && round_outer_lost(inner[b],cols,!is_left)) ++b;
    if(a==0) {
        int corner,kind;
        if(!round_partial(left,right,rows,cols,is_left,&corner,&kind) || kind!=1) return 0;
        // b 是开口下方第一个真实边线行；若整幅图都丢线则没有锚点可供补线。
        if (b <= 0 || b >= rows) return 0;
        // Current lower corner only; bounded virtual lead, never an old corner.
        *first=b>rows/3?b-rows/3:0; *end=b;
        return row>=*first;
    }
    if(a<3 || b+2>=rows || b-a<rows/ROUND_GAP_MIN_DIV ||
       (b-a)*100>rows*ROUND_GAP_MAX_PERCENT) return 0;
    for(int k=0;k<3;++k)
        if(!nav_pair(left[a-1-k],right[a-1-k],cols) ||
           !nav_pair(left[b+k],right[b+k],cols)) return 0;
    if((is_left?inner[a-1]:cols-1-inner[a-1])<cols/ROUND_CORNER_MIN_DIV ||
       (is_left?inner[b]:cols-1-inner[b])<cols/ROUND_CORNER_MIN_DIV ||
       !round_outer_continuous(outer,a-1,b,rows,cols,is_left)) return 0;
    *first=a; *end=b;
    return 1;
}

// Restored straight boundaries alone do not prove the car turned into a ring.
// Require a same-direction near/far bend supported by five real pairs per band.
static int nav_entry_turn(const uint8_t *left, const uint8_t *right,
                          int near, int far, int cols, int is_left)
{
    int delta=0;
    for(int k=-2;k<=2;++k)
    {
        if(!nav_pair(left[near+k],right[near+k],cols) ||
           !nav_pair(left[far+k],right[far+k],cols)) return 0;
        delta+=(int)left[near+k]+right[near+k]-left[far+k]-right[far+k];
    }
    return (is_left?delta:-delta)>=10*(cols/32);
}

// Seed missing row widths only from current real, consecutive boundary pairs.
// Interpolate enclosed gaps; do not extrapolate from one side or use 0/W-1.
static void nav_seed_ring_width(track_nav_t *nav, const uint8_t *left,
                                const uint8_t *right, int rows, int cols)
{
    for(int y=rows/4;y<rows;++y)
    {
        if(nav->learned[y]) continue;
        int top=y,bottom=y;
        while(top>=1 && !(nav_pair(left[top],right[top],cols) &&
                         nav_pair(left[top-1],right[top-1],cols))) --top;
        while(bottom<rows-1 && !(nav_pair(left[bottom],right[bottom],cols) &&
                               nav_pair(left[bottom+1],right[bottom+1],cols))) ++bottom;
        if(top<1 || bottom>=rows-1 || bottom-top>rows/2+4) continue;
        int wt=right[top]-left[top],wb=right[bottom]-left[bottom];
        if(wt>cols*9/10 || wb>cols*9/10 || wt-wb>cols/3 || wb-wt>cols/3) continue;
        int width=top==bottom?wt:wt+(wb-wt)*(y-top)/(bottom-top);
        if(width<cols/10) continue;
        nav->width[y]=(uint8_t)width;
        nav->learned[y]=1;
    }
}

#include "ring_vision.inc"
#include "abc_navigation.inc"

void track_nav_update(track_nav_t *nav, const track_detector_t *detector,
                      const uint8_t *raw_left, const uint8_t *raw_right, const uint8_t *binary,
                      uint8_t *left, uint8_t *right, uint8_t *mid,
                      uint8_t rows, uint8_t cols, uint32_t now_ms)
{
    // 环岛也走本导航器：入口缺口、黑色岛边和出口道路共用一套阶段/补线逻辑。
    // 视觉 A/B/C 点只保留作调试显示，不能再接管控制路径。

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
    int ring_confirmed = detector->confirmed==TRACK_ROUND_LEFT ||
                         detector->confirmed==TRACK_ROUND_RIGHT;
    if (ring_confirmed && detector->candidate==detector->confirmed &&
        (nav->phase==NAV_NORMAL || nav->phase==NAV_SLOW))
        nav_seed_ring_width(nav,raw_left,raw_right,rows,cols);
    // A stopped entry may resume only with fresh same-direction evidence and
    // visible reference edge at both control rows, within the original timeout.
    if (nav->phase==NAV_SLOW && nav->entry_pending && ring_confirmed &&
        nav->element==detector->confirmed && detector->candidate==detector->confirmed &&
        now_ms-nav->started_ms<NAV_RING_TIMEOUT_MS && nav->learned[near] && nav->learned[far])
    {
        const uint8_t *outer=nav->element==TRACK_ROUND_LEFT?raw_right:raw_left;
        if(round_outer_visible(outer[near],cols) && round_outer_visible(outer[far],cols))
            nav_stage(nav,NAV_APPROACH,now_ms);
    }
    if (nav->phase == NAV_NORMAL && detector->confirmed != TRACK_NONE &&
        detector->confirmed != TRACK_UNKNOWN)
    {
        nav->element = detector->confirmed;
        nav->entry_pending = ring_confirmed;
        nav->exit_guide_valid = 0;
        nav->started_ms = now_ms;
        nav->fault = !nav->learned[near] || !nav->learned[far] ? NAV_FAULT_WIDTH : NAV_FAULT_NONE;
        nav_stage(nav, !nav->learned[near] || !nav->learned[far] ? NAV_SLOW :
            (nav->element == TRACK_CROSS ? NAV_CROSS : NAV_APPROACH), now_ms);
    }
    uint32_t element_timeout = (nav->element == TRACK_ROUND_LEFT ||
                                nav->element == TRACK_ROUND_RIGHT) ?
                               NAV_RING_TIMEOUT_MS : NAV_TIMEOUT_MS;
    if (nav->phase != NAV_NORMAL && nav->phase != NAV_RECOVER && nav->phase != NAV_SLOW &&
        now_ms - nav->started_ms >= element_timeout)
    {
        nav->fault = NAV_FAULT_TIMEOUT;
        nav_stage(nav, NAV_SLOW, now_ms);
    }
    if (nav->phase == NAV_RECOVER && now_ms - nav->stage_ms >= NAV_RECOVER_TIMEOUT_MS)
    {
        nav->fault = NAV_FAULT_RECOVER;
        nav_stage(nav, NAV_SLOW, now_ms);
    }
    // 本次环岛方向由初次确认结果锁存到NAV_NORMAL，忽略途中反侧候选对路线的影响。
    int is_left = nav->element == TRACK_ROUND_LEFT;
    int ring_nav = nav->element == TRACK_ROUND_LEFT || nav->element == TRACK_ROUND_RIGHT;
    // 丢线故障时仍沿已锁定的环岛阶段与方向尝试恢复；状态仍保持SLOW，
    // 因此无有效路径时停车保护不变，也不会把普通弯道纳入环岛补线。
    track_nav_phase_t ring_path_phase = nav->phase;
    if (nav->phase == NAV_SLOW && ring_nav &&
        (nav->slow_resume_phase == NAV_APPROACH || nav->slow_resume_phase == NAV_ENTER ||
         nav->slow_resume_phase == NAV_INSIDE || nav->slow_resume_phase == NAV_EXIT))
        ring_path_phase = nav->slow_resume_phase;
    int inner_open = is_left ? left_open : right_open;
    int outer_open = is_left ? right_open : left_open;
    int guide_row = 0, guide_center = 0, guide_slope = 0;
    int exit_evidence = nav_exit_road(nav, raw_left, raw_right, rows, cols,
                                     is_left, &guide_row, &guide_center, &guide_slope);
    nav->exit_evidence=(uint8_t)exit_evidence;
    // EXIT端点在IN阶段首次可靠出现时缓存；轮廓短暂抖动时仍沿远端出口方向延伸。
    if (ring_path_phase == NAV_INSIDE && exit_evidence) {
        nav->exit_guide_row = (uint8_t)guide_row;
        nav->exit_guide_center = (uint8_t)guide_center;
        nav->exit_guide_slope = (int16_t)guide_slope;
        nav->exit_guide_valid = 1;
    }
    nav->guide_valid = 0;
    int feature = 0;
    if (nav->phase == NAV_APPROACH) feature = inner_open >= rows / 20;
    if (nav->phase == NAV_ENTER) feature = full && inner_open == 0 &&
        nav_entry_turn(raw_left,raw_right,near,far,cols,is_left);
    nav->exit_straight_fallback=(uint8_t)(
        ring_path_phase==NAV_INSIDE && full && straight &&
        inner_open==0 && outer_open==0 && both_open==0 &&
        now_ms-nav->stage_ms>=NAV_RING_STRAIGHT_EXIT_MIN_MS &&
        nav->straight_frames>=NAV_RING_EXIT_STRAIGHT_FRAMES &&
        detector->confirmed==TRACK_NONE &&
        (detector->candidate==TRACK_NONE || detector->candidate==TRACK_UNKNOWN));
    int ring_exit_feature = ring_path_phase == NAV_INSIDE &&
        ((outer_open >= rows / 20 && both_open == 0 && exit_evidence) ||
         nav->exit_straight_fallback);
    if (ring_path_phase == NAV_INSIDE)
    {
        // 证据短暂消失时只递减一帧，防抖；须累计8帧才接受出口。
        if (ring_exit_feature)
        { if (nav->exit_confirm_frames < NAV_RING_EXIT_CONFIRM_FRAMES) ++nav->exit_confirm_frames; }
        else if (nav->exit_confirm_frames) --nav->exit_confirm_frames;
    }
    else nav->exit_confirm_frames = 0;
    if (feature) { if (nav->feature_frames < NAV_FEATURE_FRAMES) ++nav->feature_frames; }
    else nav->feature_frames = 0;
    // 边线几何已连续、道路近远行一致时，UNKNOWN只是尚未分类，不应卡死退出恢复。
    if (straight &&
        (detector->candidate == TRACK_NONE || detector->candidate == TRACK_UNKNOWN))
    {
        uint8_t required = ring_nav ? NAV_RING_EXIT_STRAIGHT_FRAMES : NAV_EXIT_STRAIGHT_FRAMES;
        if (nav->straight_frames < required) ++nav->straight_frames;
    }
    else nav->straight_frames = 0;

    uint32_t elapsed = now_ms - nav->stage_ms;
    if (nav->phase == NAV_APPROACH && nav->feature_frames >= NAV_FEATURE_FRAMES)
        nav_stage(nav, NAV_ENTER, now_ms);
    else if (nav->phase == NAV_ENTER && elapsed >= NAV_STAGE_MIN_MS &&
             nav->feature_frames >= NAV_FEATURE_FRAMES)
        nav_stage(nav, NAV_INSIDE, now_ms);
    else if (nav->phase == NAV_INSIDE && elapsed >= NAV_INSIDE_MIN_MS &&
             nav->exit_confirm_frames >= NAV_RING_EXIT_CONFIRM_FRAMES)
    {
        nav->exit_anchor = nav->last_mid[rows - 1];
        nav_stage(nav, NAV_EXIT, now_ms);
    }
    else if (nav->phase == NAV_SLOW && ring_nav && ring_path_phase == NAV_INSIDE &&
             now_ms-nav->started_ms >= NAV_INSIDE_MIN_MS &&
             nav->exit_confirm_frames >= NAV_RING_EXIT_CONFIRM_FRAMES)
    {
        // 丢线后重获可靠出口证据，按原环岛状态进入出口阶段，不清除环岛方向。
        nav->exit_anchor = nav->last_mid[rows - 1];
        nav_stage(nav, NAV_EXIT, now_ms);
    }
    else if (nav->phase == NAV_EXIT && elapsed >= NAV_RING_EXIT_MIN_MS &&
             nav->straight_frames >= NAV_RING_EXIT_STRAIGHT_FRAMES)
        nav_stage(nav, NAV_RECOVER, now_ms);
    else if (nav->phase == NAV_CROSS && elapsed >= NAV_STAGE_MIN_MS &&
             nav->straight_frames >= NAV_EXIT_STRAIGHT_FRAMES)
        nav_stage(nav, NAV_RECOVER, now_ms);
    else if ((nav->phase == NAV_RECOVER || nav->phase == NAV_SLOW) &&
             elapsed >= (ring_nav ? NAV_RING_RECOVER_MS : NAV_RECOVER_MS) &&
             nav->straight_frames >= (ring_nav ? NAV_RING_EXIT_STRAIGHT_FRAMES : NAV_EXIT_STRAIGHT_FRAMES) &&
             detector->confirmed == TRACK_NONE &&
             (!ring_nav || (nav->phase == NAV_RECOVER || nav->slow_resume_phase == NAV_EXIT)))
    {
        nav_stage(nav, NAV_NORMAL, now_ms);
        nav->fault = NAV_FAULT_NONE;
        nav->element = TRACK_NONE;
    }

    if (nav->phase == NAV_EXIT ||
        (nav->phase == NAV_SLOW && ring_path_phase == NAV_EXIT))
    {
        // 开口消失后优先更新真实出口证据；短暂丢失时沿缓存远端道路继续走。
        int fresh_guide = exit_evidence || nav_far_road(nav, raw_left, raw_right,
            rows, cols, far, &guide_row, &guide_center, &guide_slope);
        if (fresh_guide) {
            nav->exit_guide_row = (uint8_t)guide_row;
            nav->exit_guide_center = (uint8_t)guide_center;
            nav->exit_guide_slope = (int16_t)guide_slope;
            nav->exit_guide_valid = 1;
        } else if (nav->exit_guide_valid) {
            guide_row = nav->exit_guide_row;
            guide_center = nav->exit_guide_center;
            guide_slope = nav->exit_guide_slope;
        }
        nav->guide_valid = fresh_guide || nav->exit_guide_valid;
    }
    if (nav->phase == NAV_CROSS)
    {
        nav->guide_valid = nav_far_road(nav, raw_left, raw_right, rows, cols,
            far, &guide_row, &guide_center, &guide_slope);
        nav_bridge(left, rows, cols, 1);
        nav_bridge(right, rows, cols, 0);
    }
    nav_ring_apex(nav, binary, rows, cols, is_left, now_ms);
    uint8_t island_edge[256];
    nav_ring_contour(nav, binary, rows, cols, is_left, island_edge);
    int entry_first=0,entry_end=0;
    int entry_open=ring_path_phase==NAV_ENTER &&
        nav_entry_gap(raw_left,raw_right,near,rows,cols,is_left,&entry_first,&entry_end);
    nav->entry_guide=0;
    nav->entry_gap_valid=(uint8_t)entry_open;
    nav->entry_gap_rows=0;
    nav->entry_edge_rows=0;
    nav->entry_guided_rows=0;
    nav->exit_reject_rows=0;
    uint32_t path_hold_ms=(ring_path_phase==NAV_INSIDE || ring_path_phase==NAV_EXIT) ?
                          NAV_INSIDE_HOLD_MS : NAV_HOLD_MS;
    for (int y = rows / 4; y < rows; ++y)
    {
        int usable = nav_pair(left[y], right[y], cols);
        int center = ((int)left[y] + right[y]) / 2;
        int width = nav->width[y];
        int ring = ring_nav && (ring_path_phase == NAV_APPROACH || ring_path_phase == NAV_ENTER ||
                   ring_path_phase == NAV_INSIDE || ring_path_phase == NAV_EXIT);
        if (ring && ring_path_phase != NAV_EXIT && nav->learned[y])
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
            // 入口默认沿外侧边线；确认岛边后修补内边线，环内沿岛边。
            if (ring_path_phase == NAV_INSIDE && inner_valid)
            {
                center = is_left ? raw_left[y] + width / 2 : raw_right[y] - width / 2;
                usable = 1;
            }
            else if (outer_valid)
            {
                center = is_left ? outer_position - width / 2 : outer_position + width / 2;
                usable = 1;
            }
            if (entry_open && usable && !island_edge[y] &&
                y>=entry_first && y<entry_end)
            {
                // Repair the opening-side boundary toward the opening, using
                // its real lower corner. Bound the resulting center shift.
                int anchor=is_left?raw_left[entry_end]:raw_right[entry_end];
                int border=is_left?ROUND_EDGE_MARGIN:cols-1-ROUND_EDGE_MARGIN;
                int boundary=anchor+(border-anchor)*(entry_end-y)/(entry_end-entry_first);
                int target=is_left?boundary+width/2:boundary-width/2;
                int shift=is_left?center-target:target-center;
                shift=nav_clip(shift,0,width/NAV_ENTRY_SHIFT_DIV);
                target=center+(is_left?-shift:shift);
                if (shift>0 && target-width/2>0 && target+width/2<cols-1 &&
                    (!binary || binary[y*cols+target]!=0))
                { center=target; ++nav->entry_guided_rows;
                  if(y==near) nav->entry_guide=1; }
            }
            if (entry_open && usable && !island_edge[y] &&
                y>=entry_first && y<entry_end)
                ++nav->entry_gap_rows;
            if (island_edge[y] &&
                (ring_path_phase==NAV_APPROACH || ring_path_phase==NAV_ENTER))
            {
                int boundary=island_edge[y];
                int corridor=outer_valid ? (is_left?outer_position-boundary:
                                                    boundary-outer_position) : width;
                int candidate=is_left?boundary+corridor/2:boundary-corridor/2;
                if (corridor>=cols/10 && candidate-corridor/2>0 &&
                    candidate+corridor/2<cols-1)
                { center=candidate; width=corridor; usable=1;
                  ++nav->entry_edge_rows;
                  if(y==near) nav->entry_guide=2; }
            }
            // A current black pixel cannot be accepted as a drivable center,
            // including while waiting for temporal cap confirmation.
            if (usable && binary &&
                (ring_path_phase==NAV_APPROACH || ring_path_phase==NAV_ENTER) &&
                (center<=0 || center>=cols-1 || binary[y*cols+center]==0)) usable=0;
            if (usable)
            {
                left[y] = (uint8_t)nav_clip(center - width / 2, 1, cols - 2);
                right[y] = (uint8_t)nav_clip(center + width / 2, 1, cols - 2);
                usable = nav_pair(left[y], right[y], cols);
            }
        }
        if (nav->phase == NAV_EXIT ||
            (nav->phase == NAV_SLOW && ring_nav && ring_path_phase == NAV_EXIT))
        {
            usable = 0;
            if (nav->guide_valid && nav->learned[y] && y >= guide_row)
            {
                // 当前出口道路连接至入EXIT时车前路径；合成点必须落在白色可行驶区，
                // 否则不能把穿过黑色岛体/赛道外区域的错误转向当作有效路径。
                center = guide_center + ((int)nav->exit_anchor - guide_center) *
                         (y-guide_row) / (rows-1-guide_row);
                usable = center-width/2 > 0 && center+width/2 < cols-1;
                if (usable && binary && binary[y*cols+center] == 0)
                { usable=0; ++nav->exit_reject_rows; }
                if (usable) { left[y] = (uint8_t)(center-width/2);
                              right[y] = (uint8_t)(center+width/2); }
            }
            if (!usable && nav_pair(raw_left[y],raw_right[y],cols))
            {
                // 合成出口线未通过图像可行驶区校验时，回退到本行真实双边线。
                left[y]=raw_left[y]; right[y]=raw_right[y];
                center=((int)raw_left[y]+raw_right[y])/2;
                usable=1;
            }
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
            (nav->phase == NAV_CROSS || nav->phase == NAV_RECOVER ||
             (nav->phase == NAV_SLOW &&
              !(ring_nav && (ring_path_phase == NAV_APPROACH || ring_path_phase == NAV_ENTER ||
                             ring_path_phase == NAV_INSIDE || ring_path_phase == NAV_EXIT)))))
        {
            if (left[y] == 0 && right[y] > 0 && right[y] < cols - 1)
                left[y] = (uint8_t)nav_clip(right[y] - width, 1, cols - 2);
            else if (right[y] == cols - 1 && left[y] > 0 && left[y] < cols - 1)
                right[y] = (uint8_t)nav_clip(left[y] + width, 1, cols - 2);
            usable = nav_pair(left[y], right[y], cols);
            center = ((int)left[y] + right[y]) / 2;
        }
        // 真实/补线可用性与短时保持分开，保持不能刷新last_path_ms。
        if(y==near && !usable) nav->entry_guide=0;
        nav->path_valid[y] = usable;
        if (usable)
        {
            if (!ring) center = ((int)left[y] + right[y]) / 2;
            mid[y] = (uint8_t)nav_clip(center, 1, cols - 2);
            nav->last_mid[y] = mid[y];
        }
        else if (nav->phase != NAV_NORMAL && nav->learned[y] &&
                 now_ms - nav->last_path_ms <= path_hold_ms)
            mid[y] = nav->last_mid[y];
    }
    if (nav->path_valid[near] && nav->path_valid[far]) nav->last_path_ms = now_ms;
    nav->drive_valid = (nav->path_valid[near] && nav->path_valid[far]) ||
        (nav->phase != NAV_NORMAL && now_ms-nav->last_path_ms <= path_hold_ms);
    if (nav->phase != NAV_NORMAL && nav->phase != NAV_SLOW &&
             now_ms - nav->last_path_ms > path_hold_ms)
    {
        nav->fault = NAV_FAULT_PATH;
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


