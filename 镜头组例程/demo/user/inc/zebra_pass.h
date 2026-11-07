#ifndef ZEBRA_PASS_H
#define ZEBRA_PASS_H
#include <stdint.h>
#include <math.h>

#define ZEBRA_CONFIRM_FRAMES       3u
#define ZEBRA_CLEAR_FRAMES         5u
#define ZEBRA_STRAIGHT_MIN_MS     200u
#define ZEBRA_STRAIGHT_MAX_MS    1500u
#define ZEBRA_RECOVER_MS          300u
#define ZEBRA_STABLE_PIXELS         4
#define ZEBRA_PASS_SPEED         125.0f
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
// 统计采样线上黑白跳变次数，作为斑马线条纹密度的基础量。
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

// Four complete black bars, with regular white gaps and aligned probes across 20 pixels.
// Initial pixel limits for 188x120; camera images are still needed for calibration.
#define ZEBRA_BAR_COUNT 4
#define ZEBRA_PROBE_COUNT 21
#define ZEBRA_PROBE_MIN_GOOD 15
#define ZEBRA_PROBE_MAX_BAD_RUN 2
// 从一条扫描线上提取多组黑条起止位置；少于规定数量或宽度非法时拒绝该扫描线。
static int zebra_bars(const uint8_t *p, unsigned n, unsigned step,
                      int start, int *lo, int *hi)
{
    int pos=start;
    // Skip a bar clipped by the scan start; later complete bars may still qualify.
    if(pos>0 && pos<(int)n && p[(pos-1)*step]==0)
        while(pos<(int)n && p[pos*step]==0) ++pos;
    for(int k=0;k<ZEBRA_BAR_COUNT;++k) {
        while(pos<(int)n && p[pos*step]!=0) ++pos;
        if(pos==0 || pos>=(int)n || p[(pos-1)*step]==0) return 0; // White leading margin.
        lo[k]=pos;
        while(pos<(int)n && p[pos*step]==0) ++pos;
        if(pos>=(int)n) return 0; // No clipped black bars.
        hi[k]=pos;
    }
    return 1;
}
// 检查提取到的黑条宽度、间隔是否落在允许变化范围，抑制随机纹理误报。
static int zebra_regular(const int *lo,const int *hi,int span)
{
    int minw=1000,maxw=0,ming=1000,maxg=0;
    for(int k=0;k<ZEBRA_BAR_COUNT;++k) {
        int width=hi[k]-lo[k];
        if(width<2 || width*2>span) return 0; // Bars must be elongated.
        if(width<minw) minw=width;
        if(width>maxw) maxw=width;
        if(k) {
            int gap=lo[k]-hi[k-1];
            if(gap<2) return 0;
            if(gap<ming) ming=gap;
            if(gap>maxg) maxg=gap;
        }
    }
    return maxw<=minw*2 && maxg<=ming*2 && maxg<=minw*3 && maxw<=ming*3;
}
// 在多个邻近扫描位置重复验证条纹节奏；要求足够多探测线通过且连续失败不能过长。
static int zebra_patch(const uint8_t *base,unsigned length,unsigned step,unsigned probe_step)
{
    int lo[4],hi[4],prev_lo[4],prev_hi[4];
    // 允许21个相邻探测位置中少量污损，但只跟踪同一组条纹，避免噪点凑数。
    for(int seed=0;seed<=ZEBRA_PROBE_MAX_BAD_RUN;++seed) {
        const uint8_t *first=base+seed*probe_step;
        for(int begin=1;begin<(int)length-14;) {
            if(!zebra_bars(first,length,step,begin,lo,hi)) break;
            begin=hi[0]+1;
            if(!zebra_regular(lo,hi,20)) continue;
            for(int k=0;k<4;++k) {prev_lo[k]=lo[k];prev_hi[k]=hi[k];}
            int good=0,bad=seed,bad_run=0,last_good=seed,line;
            for(line=seed;line<ZEBRA_PROBE_COUNT;++line) {
                const uint8_t *p=base+line*probe_step;
                int gap=line-last_good;
                int from=prev_lo[0]-2*gap;
                if(from<1) from=1;
                int ok=zebra_bars(p,length,step,from,lo,hi) && zebra_regular(lo,hi,20);
                if(ok && line>last_good) {
                    int shift=lo[0]-prev_lo[0],period=prev_lo[1]-prev_lo[0];
                    if(shift < -2*gap || shift > 2*gap ||
                       shift*2<=-period || shift*2>=period) ok=0;
                    for(int k=0;k<4 && ok;++k) {
                        int d=lo[k]-prev_lo[k]-shift,e=hi[k]-prev_hi[k]-shift;
                        if(d < -gap || d > gap || e < -gap || e > gap ||
                           lo[k]>=prev_hi[k] || hi[k]<=prev_lo[k]) ok=0;
                    }
                }
                if(ok) {
                    ++good;bad_run=0;last_good=line;
                    for(int k=0;k<4;++k) {prev_lo[k]=lo[k];prev_hi[k]=hi[k];}
                } else {
                    ++bad;++bad_run;
                    if(bad>ZEBRA_PROBE_COUNT-ZEBRA_PROBE_MIN_GOOD ||
                       bad_run>ZEBRA_PROBE_MAX_BAD_RUN) break;
                }
            }
            if(line==ZEBRA_PROBE_COUNT && good>=ZEBRA_PROBE_MIN_GOOD) return 1;
        }
    }
    return 0;
}
// 在水平与垂直方向寻找规则条纹区域，并结合输入边线限定有效道路区域。
static uint8_t zebra_detect_lane(const uint8_t *image,unsigned h,unsigned w,
                                  const uint8_t *left,const uint8_t *right)
{
    if(!image || h<60 || w<80) return 0;
    unsigned x0=w/4,x1=w*3/4; // 中央车道ROI，兼容车辆横向偏移。
    if(left && right) {
        unsigned l=0,r=0,count=0,minwidth=w,maxwidth=0;
        for(unsigned y=h-5;y<h;++y) {
            if(left[y]<=2 || right[y]>=w-3 || right[y]<=left[y]+w/4) continue;
            unsigned width=right[y]-left[y];
            if(width<minwidth) minwidth=width;
            if(width>maxwidth) maxwidth=width;
            l+=left[y];r+=right[y];++count;
        }
        if(count>=3 && maxwidth-minwidth<=w/16) {
            l/=count;r/=count;
            x0=l+(r-l)/8;x1=r-(r-l)/8;
            if(x0<w/4) x0=w/4;
            if(x1>w*3/4) x1=w*3/4;
        }
    }
    if(x1<=x0+20) return 0;
    // Consecutive rows spanning 20 pixels: repeated vertical bars in the camera image.
    for(unsigned y=h/3;y+20<h*9/10;y+=2)
        if(zebra_patch(image+y*w+x0,x1-x0,1,w)) return 1;
    // Consecutive columns spanning 20 pixels: transverse bars. Position matching is
    // mandatory; independent transition counts in different columns are not enough.
    for(unsigned x=x0;x+20<x1;x+=4)
        if(zebra_patch(image+(h/3)*w+x,h*9/10-h/3,w,1)) return 1;
    return 0;
}
static uint8_t zebra_detect(const uint8_t *image,unsigned h,unsigned w)
{ return zebra_detect_lane(image,h,w,0,0); }

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

// 判断指定行边界是否真实、宽度合理且不被条纹污染，用于恢复普通循迹路径。
static uint8_t zebra_row_valid(const uint8_t *l, const uint8_t *r,
                               const uint8_t *bad, unsigned y, unsigned w)
{
    return !bad[y] && l[y] > 0 && r[y] < w - 1 &&
           r[y] > l[y] && (unsigned)(r[y] - l[y]) >= w / 6;
}

static void zebra_update(zebra_pass_t *s, uint8_t candidate, uint8_t path_valid,
                         uint8_t near_mid, uint8_t far_mid, uint32_t now)
{
    // candidate 必须同时满足斑马线图像特征和车道补线有效，避免仅凭条纹误接管。
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
    // 恢复阶段依赖稳定清路证据退出，不等待旧的舵机更新标志，避免状态永久卡住。
    else if ((uint32_t)(now - s->phase_ms) >= ZEBRA_RECOVER_MS &&
             s->clear >= ZEBRA_CLEAR_FRAMES)
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

// 候选或任一非普通阶段都要求使用斑马线专用限速。
static uint8_t zebra_slow(const zebra_pass_t *s, uint8_t candidate)
{
    return candidate || s->phase != ZEBRA_NORMAL;
}

// 旧版舵角渐变接口保留给兼容调用；当前主循环使用zebra_lane_steering完成相同职责。
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

/* 一次看见斑马线只计一次；离开条纹后重新布防第二次计数。 */
typedef enum { ZEBRA_RACE_COUNT=0, ZEBRA_RACE_ALIGN, ZEBRA_RACE_STOP } zebra_race_phase_t;
#define ZEBRA_REARM_CLEAR_FRAMES 12u
#define ZEBRA_REARM_CLEAR_COUNTS ZEBRA_FINISH_CLEAR_COUNTS
#define ZEBRA_ALIGN_TOL_DEG 8.0f
#define ZEBRA_ALIGN_STABLE_FRAMES 8u
#define ZEBRA_ALIGN_GAIN 0.25f
#define ZEBRA_ALIGN_MAX_STEER 22.0f

/* 从重复条纹像素求主轴；车头朝向在前视图中近似图像竖直方向。 */
static uint8_t zebra_axis_angle(const uint8_t *im,unsigned h,unsigned w,float *deg)
{
 if(!im||!deg||h<40||w<40||h>240||w>320)return 0;
 unsigned x0=w/4,x1=w*3/4,y0=h/3,y1=h*9/10,rw=x1-x0,rh=y1-y0;
 unsigned rs=0,cs=0;
 for(unsigned y=y0;y<y1;++y){unsigned t=0,bl=0;uint8_t p=im[y*w+x0];for(unsigned x=x0;x<x1;++x){uint8_t v=im[y*w+x];bl+=(v==0);if(x!=x0&&v!=p)++t;p=v;}if(t>=4||bl>rw/4)rs+=t+1;}
 for(unsigned x=x0;x<x1;++x){unsigned t=0,bl=0;uint8_t p=im[y0*w+x];for(unsigned y=y0;y<y1;++y){uint8_t v=im[y*w+x];bl+=(v==0);if(y!=y0&&v!=p)++t;p=v;}if(t>=4||bl>rh/4)cs+=t+1;}
 uint8_t rows=rs>=cs;if((rows&&rs<20)||(!rows&&cs<20))return 0;
 float sx=0,sy=0,sxx=0,syy=0,sxy=0;unsigned n=0;
 if(rows){for(unsigned y=y0;y<y1;++y){unsigned t=0,bl=0;uint8_t p=im[y*w+x0];for(unsigned x=x0;x<x1;++x){uint8_t v=im[y*w+x];bl+=(v==0);if(x!=x0&&v!=p)++t;p=v;}if(t<4&&bl<=rw/4)continue;for(unsigned x=x0;x<x1;++x)if(im[y*w+x]==0){sx+=x;sy+=y;sxx+=(double)x*x;syy+=(double)y*y;sxy+=(double)x*y;++n;}}}
 else{for(unsigned x=x0;x<x1;++x){unsigned t=0,bl=0;uint8_t p=im[y0*w+x];for(unsigned y=y0;y<y1;++y){uint8_t v=im[y*w+x];bl+=(v==0);if(y!=y0&&v!=p)++t;p=v;}if(t<4&&bl<=rh/4)continue;for(unsigned y=y0;y<y1;++y)if(im[y*w+x]==0){sx+=x;sy+=y;sxx+=(double)x*x;syy+=(double)y*y;sxy+=(double)x*y;++n;}}}
 if(n<24)return 0;float xx=sxx-sx*sx/n,yy=syy-sy*sy/n,xy=sxy-sx*sy/n;
 *deg=0.5f*atan2f(2.0f*xy,xx-yy)*57.2957795f;return 1;
}
typedef struct { zebra_race_phase_t phase; uint8_t initialized,sighting_count,candidate_hits,clear_frames,latched,aligned_frames,angle_valid,angle_initialized; int8_t target_sign; uint32_t last_ms,phase_ms,last_sighting_counts; float stripe_angle_deg,last_stripe_angle_deg,angle_error_deg; } zebra_race_t;
static void zebra_race_update(zebra_race_t *r,const zebra_pass_t *pass,uint8_t candidate,const uint8_t *im,unsigned h,unsigned w,uint32_t now,uint32_t distance_counts)
{
 (void)pass;if(r->phase==ZEBRA_RACE_STOP)return;if(!r->initialized){r->initialized=1;r->last_ms=now;}
 if((uint32_t)(now-r->last_ms)>=ZEBRA_FRAME_GAP_MS){r->candidate_hits=0;r->clear_frames=0;r->aligned_frames=0;}r->last_ms=now;
 if(r->phase==ZEBRA_RACE_COUNT){if(candidate){r->clear_frames=0;if(!r->latched&&r->candidate_hits<ZEBRA_CONFIRM_FRAMES)++r->candidate_hits;if(!r->latched&&r->candidate_hits>=ZEBRA_CONFIRM_FRAMES){r->latched=1;r->candidate_hits=0;r->last_sighting_counts=distance_counts;if(r->sighting_count<2)++r->sighting_count;if(r->sighting_count>=2){r->phase=ZEBRA_RACE_ALIGN;r->phase_ms=now;r->aligned_frames=0;}}}else{r->candidate_hits=0;if(r->clear_frames<ZEBRA_REARM_CLEAR_FRAMES)++r->clear_frames;if(r->latched&&r->clear_frames>=ZEBRA_REARM_CLEAR_FRAMES&&(int32_t)(distance_counts-r->last_sighting_counts)>=(int32_t)ZEBRA_REARM_CLEAR_COUNTS){r->latched=0;r->clear_frames=0;}}return;}
 if(r->phase==ZEBRA_RACE_ALIGN){r->angle_valid=zebra_axis_angle(im,h,w,&r->stripe_angle_deg);if(!r->angle_valid){r->aligned_frames=0;return;}if(!r->angle_initialized){r->target_sign=(r->stripe_angle_deg<0.0f)?-1:1;r->last_stripe_angle_deg=r->stripe_angle_deg;r->angle_initialized=1;}else{while(r->stripe_angle_deg-r->last_stripe_angle_deg>90.0f)r->stripe_angle_deg-=180.0f;while(r->stripe_angle_deg-r->last_stripe_angle_deg< -90.0f)r->stripe_angle_deg+=180.0f;r->last_stripe_angle_deg=r->stripe_angle_deg;}float target=(float)r->target_sign*90.0f;r->angle_error_deg=target-r->stripe_angle_deg;if(r->angle_error_deg>ZEBRA_ALIGN_TOL_DEG||r->angle_error_deg< -ZEBRA_ALIGN_TOL_DEG)r->aligned_frames=0;else if(r->aligned_frames<ZEBRA_ALIGN_STABLE_FRAMES)++r->aligned_frames;if(r->aligned_frames>=ZEBRA_ALIGN_STABLE_FRAMES)r->phase=ZEBRA_RACE_STOP;}
}
static float zebra_align_steering(const zebra_race_t *r,float center,float previous,float direction)
{if(!r->angle_valid)return previous;float d=r->angle_error_deg*ZEBRA_ALIGN_GAIN*direction;if(d>ZEBRA_ALIGN_MAX_STEER)d=ZEBRA_ALIGN_MAX_STEER;if(d< -ZEBRA_ALIGN_MAX_STEER)d= -ZEBRA_ALIGN_MAX_STEER;return center+d;}

// 将通过阶段映射为定长LCD提示，状态文本长度需适配屏幕字符宽度。
static const char *zebra_status(const zebra_pass_t *s, const zebra_race_t *race,
                               uint8_t candidate)
{
    if (race->phase == ZEBRA_RACE_STOP) return "Z:ALIGNED STOP       ";
    if (race->phase == ZEBRA_RACE_ALIGN) return "Z:ALIGN BLACK BAR   ";
    if (race->sighting_count == 1) return candidate ? "Z:SEEN 1 CHECK      " : "Z:SEEN 1 WAIT NEXT  ";
    switch (s->phase)
    {
        case ZEBRA_STRAIGHT: return "Z:PASS 1 STRIPE     ";
        case ZEBRA_RECOVER:  return "Z:RECOVER 1         ";
        case ZEBRA_TIMEOUT:  return "Z:TIMEOUT 1         ";
        default: return candidate ? "Z:CHECK STRIPE      " : "Z:COUNT STRIPES     ";
    }
}
#endif
