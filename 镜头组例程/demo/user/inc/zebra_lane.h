#ifndef SMARTCAR_ZEBRA_LANE_H
#define SMARTCAR_ZEBRA_LANE_H
#include <stdint.h>

/* 二值图中道路为白色、背景为黑色。
 * 独立于容易受条纹干扰的中心向外扫描，重建道路外侧边界。
 * 宽度数组和输入图像只读。 */
#define ZEBRA_LANE_ROWS_MAX 256u
#define ZEBRA_LANE_SUPPORT 8
#define ZEBRA_LANE_FIT_ROWS 12
#define ZEBRA_LANE_HOLD_MS 200u
#define ZEBRA_LANE_PRIOR_MS 300u
#define ZEBRA_LANE_FOLLOW_DPS 90.0f
#define ZEBRA_LANE_RETURN_DPS 20.0f
#define ZEBRA_LANE_STEP_MAX_MS 50u

typedef struct {
    uint8_t current;
    uint8_t prior_valid[ZEBRA_LANE_ROWS_MAX],prior_mid[ZEBRA_LANE_ROWS_MAX];
    uint8_t prior_width_valid[ZEBRA_LANE_ROWS_MAX],prior_width[ZEBRA_LANE_ROWS_MAX];
    uint8_t prior_ready,steering_ready,active,recovering;
    uint32_t prior_ms,steering_ms,last_good_ms;
    float angle;
} zebra_lane_t;

static int zebra_lane_abs(int x) {return x<0?-x:x;}
static float zebra_lane_absf(float x) {return x<0.0f?-x:x;}

/* 返回朝向道路的黑色像素。有限的内部斑马线条纹外侧是白色，
 * 无法满足边界外侧必须为黑色的条件。 */
static int zebra_lane_outer(const uint8_t *row,unsigned w,int side,int expected)
{
    int black=0,best=-1,best_score=10000;
    for(int q=0;q<(int)w-3;++q) {
        int x=side?(int)w-1-q:q;
        if(row[x]==0) {++black;continue;}
        if(q<3) continue;
        // 局部暗色边界可抵抗画面反光；仍要求朝图像边缘方向有一定黑色背景，
        // 避免把孤立的中央条纹误当成道路边界。
        int band=q<16?q:16,local_black=0;
        for(int k=1;k<=band;++k) {
            int outside=side?x+k:x-k;
            if(row[outside]==0) ++local_black;
        }
        if(local_black*5<band*4 || black*4<q) continue;
        int inward=side?-1:1;
        if(row[x+inward]==0 || row[x+2*inward]==0 ||
           row[x-inward]!=0 || row[x-2*inward]!=0 || row[x-3*inward]!=0) continue;
        int edge=x-inward;
        int score=zebra_lane_abs(edge-expected);
        if(score<best_score) {best=edge;best_score=score;}
    }
    return best;
}

/* 至少八个连续的当前帧采样点才构成边界线段。
 * 边界突跳会切断支撑，短斑点或岔路不能成为道路边界。 */
static void zebra_lane_support(const int16_t *raw,uint8_t *supported,int h,int first)
{
    int start=first;
    for(int y=first;y<=h;++y) {
        int split=y==h || raw[y]<0 ||
            (y>start && zebra_lane_abs(raw[y]-raw[y-1])>3);
        if(split) {
            if(y-start>=ZEBRA_LANE_SUPPORT)
                for(int k=start;k<y;++k) supported[k]=1;
            start=y;
            if(y<h && raw[y]<0) start=y+1;
        }
    }
}

/* 优先使用真实支撑点，否则从附近的上方线段投影。
 * 投影点可能超出画面；调用者需加上标定半车道宽后检查中心位置。
 * 合成点不得作为新的真实证据。 */
static int zebra_lane_project(const int16_t *raw,const uint8_t *supported,
                              int target,int h,int first,float *value)
{
    if(supported[target]) {*value=(float)raw[target];return 1;}
    int last=target-1;
    while(last>=first && !supported[last]) --last;
    if(last<first || target-last>h/3) return 0;
    int count=0;
    float st=0.0f,sx=0.0f,stt=0.0f,stx=0.0f;
    for(int y=last;y>=first && count<ZEBRA_LANE_FIT_ROWS;--y) {
        if(!supported[y]) break;
        float t=(float)(y-last),x=(float)raw[y];
        st+=t;sx+=x;stt+=t*t;stx+=t*x;++count;
    }
    if(count<ZEBRA_LANE_SUPPORT) return 0;
    float denominator=(float)count*stt-st*st;
    if(denominator<=0.0f) return 0;
    float slope=((float)count*stx-st*sx)/denominator;
    float intercept=(sx-slope*st)/(float)count;
    if(zebra_lane_absf(slope)>3.0f) return 0;
    float error2=0.0f;
    for(int k=0;k<count;++k) {
        float error=intercept-slope*(float)k-(float)raw[last-k];
        if(zebra_lane_absf(error)>3.0f) return 0;
        error2+=error*error;
    }
    if(error2>(float)count*4.0f) return 0;
    *value=intercept+slope*(float)(target-last);
    return 1;
}

static void zebra_lane_build(zebra_lane_t *s,const uint8_t *image,
    const uint8_t *width,const uint8_t *learned,uint8_t *mid,uint8_t *valid,
    unsigned h,unsigned w,uint32_t now)
{
    if(!s) return;
    s->current=0;
    if(!mid || !valid || !h || h>ZEBRA_LANE_ROWS_MAX || !w || w>256u) return;
    int16_t raw_l[ZEBRA_LANE_ROWS_MAX],raw_r[ZEBRA_LANE_ROWS_MAX];
    uint8_t supported_l[ZEBRA_LANE_ROWS_MAX],supported_r[ZEBRA_LANE_ROWS_MAX];
    for(unsigned y=0;y<h;++y) {
        raw_l[y]=raw_r[y]=-1;supported_l[y]=supported_r[y]=0;
        mid[y]=(uint8_t)(w/2);valid[y]=0;
    }
    if(!image || !width || !learned || h<40u || w<60u) return;
    int first=(int)h/4;
    int prior=s->prior_ready && (uint32_t)(now-s->prior_ms)<=ZEBRA_LANE_PRIOR_MS;
    for(int y=first;y<(int)h;++y) {
        int calibrated=learned[y] && width[y]>=(int)w/10 && width[y]<=(int)w*9/10;
        int expected_width=calibrated?width[y]:
            (prior && s->prior_width_valid[y]?s->prior_width[y]:(int)w*3/4);
        int center=prior && s->prior_valid[y]?s->prior_mid[y]:(int)w/2;
        const uint8_t *row=image+(unsigned)y*w;
        raw_l[y]=(int16_t)zebra_lane_outer(row,w,0,center-expected_width/2);
        raw_r[y]=(int16_t)zebra_lane_outer(row,w,1,center+expected_width/2);
    }
    zebra_lane_support(raw_l,supported_l,(int)h,first);
    zebra_lane_support(raw_r,supported_r,(int)h,first);
    for(int y=first;y<(int)h;++y) {
        int calibrated=learned[y] && width[y]>=(int)w/10 && width[y]<=(int)w*9/10;
        int lane_width=calibrated?width[y]:
            (prior && s->prior_width_valid[y]?s->prior_width[y]:0);
        float l=0.0f,r=0.0f,center;
        int left=zebra_lane_project(raw_l,supported_l,y,(int)h,first,&l);
        int right=zebra_lane_project(raw_r,supported_r,y,(int)h,first,&r);
        if(left && right) {
            float span=r-l;
            if(lane_width && (span<(float)lane_width*0.75f || span>(float)lane_width*1.25f)) continue;
            if(!lane_width && (span<(float)w*0.25f || span>(float)w*0.9f)) continue;
            lane_width=(int)(span+0.5f);
            center=(l+r)*0.5f;
        } else if(left && lane_width) center=l+(float)lane_width*0.5f;
        else if(right && lane_width) center=r-(float)lane_width*0.5f;
        else continue;
        if(center<1.0f || center>(float)w-2.0f) continue;
        mid[y]=(uint8_t)(center+0.5f);valid[y]=1;
        if(lane_width>=(int)w/10 && lane_width<=(int)w*9/10) {
            s->prior_width[y]=(uint8_t)lane_width;
            s->prior_width_valid[y]=1;
        }
    }
    s->current=(uint8_t)(valid[h*4/5] && valid[h*3/5]);
    if(s->current) {
        for(unsigned y=0;y<h;++y) {
            s->prior_valid[y]=valid[y];
            if(valid[y]) s->prior_mid[y]=mid[y];
        }
        s->prior_ms=now;s->prior_ready=1;
    }
}

static float zebra_lane_steering(zebra_lane_t *s,float target,uint8_t guide_valid,
    uint8_t active,uint32_t now,float previous_angle,float neutral)
{
    if(!s) return target;
    if(!s->steering_ready) {
        s->steering_ready=1;s->angle=previous_angle;
        s->steering_ms=now;s->last_good_ms=now;
    }
    uint32_t elapsed=now-s->steering_ms;
    if(elapsed>ZEBRA_LANE_STEP_MAX_MS) elapsed=ZEBRA_LANE_STEP_MAX_MS;
    if(active && !s->active) {
        s->angle=previous_angle;
        s->last_good_ms=now;s->recovering=0;
    }
    if(!active && s->active) s->recovering=1;
    s->active=active!=0;
    float desired=target,rate=ZEBRA_LANE_FOLLOW_DPS;
    if(active) {
        if(guide_valid) s->last_good_ms=now;
        else if((uint32_t)(now-s->last_good_ms)<=ZEBRA_LANE_HOLD_MS)
            desired=s->angle;
        else {desired=neutral;rate=ZEBRA_LANE_RETURN_DPS;}
    } else if(!s->recovering) {
        s->angle=target;s->steering_ms=now;return target;
    }
    float step=rate*(float)elapsed/1000.0f;
    float delta=desired-s->angle;
    if(delta>step) delta=step;
    if(delta< -step) delta= -step;
    s->angle+=delta;
    s->steering_ms=now;
    if(!active && zebra_lane_absf(target-s->angle)<0.001f) s->recovering=0;
    return s->angle;
}
#endif
