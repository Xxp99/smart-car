#ifndef BRICK_AVOID_H
#define BRICK_AVOID_H
#include <stdint.h>
#include <string.h>

#ifndef BRICK_ENABLE
#define BRICK_ENABLE 1
#endif
#define BRICK_SPEED 150.0f
#define BRICK_CONFIRM_FRAMES 3
#define BRICK_CLEAR_FRAMES 5
// 2026-11-01实测：前进1米对应的编码器计数。
#define BRICK_COUNTS_PER_METER 11000u
// 若红砖的视觉特征提前消失，清空距离仍计入红砖长度。
// 33厘米是暂定的盲区余量，并非实测盲区长度。
// 转弯或打滑时，编码器行程不等于准确的纵向净空距离。
#define BRICK_CAMERA_TO_REAR_CM 22u
#define BRICK_LENGTH_CM 27u
#define BRICK_BLIND_MARGIN_CM 33u // 总清空距离82厘米，约9020个计数。
#define BRICK_CLEAR_DISTANCE_CM (BRICK_CAMERA_TO_REAR_CM + BRICK_LENGTH_CM + BRICK_BLIND_MARGIN_CM)
#define BRICK_CLEAR_COUNTS ((BRICK_COUNTS_PER_METER * BRICK_CLEAR_DISTANCE_CM + 99u) / 100u)
#define BRICK_CLEAR_MIN_MS 600u
#define BRICK_FRAME_GAP_MS 300u
#define BRICK_PATH_CONFIRM_FRAMES 3u
#define BRICK_STEER_RATE 45.0f
#define BRICK_SHIFT_RATE 60.0f // 每秒图像像素偏移量。

typedef enum {BRICK_IDLE,BRICK_AVOID,BRICK_HOLD,BRICK_RECOVER,BRICK_FAULT} brick_phase_t;
// 保存候选侧、确认/清空帧、避障偏移和故障原因，避免单帧检测直接改道。
typedef struct {
    brick_phase_t phase;
    uint8_t side,candidate,hits,clear,valid,initialized;
    uint8_t top,bottom,inset,path_hits,reason;
    uint32_t started_ms,last_ms,clear_ms,clear_counts,travel;
    float offset,target;
} brick_t;
enum {BRICK_OK,BRICK_WIDTH,BRICK_EDGE,BRICK_NARROW,BRICK_SPACE,BRICK_BLACK,BRICK_WAIT,BRICK_FAILED,BRICK_ONE};
static const char *brick_reason(const brick_t *s) {
    static const char *const names[]={"OK","WIDTH","EDGE","NARROW","SPACE","BLACK","WAIT","FAULT","ONE"};
    return names[s->reason<=BRICK_ONE?s->reason:BRICK_FAILED];
}
typedef struct {float angle;uint32_t ms;uint8_t initialized;} brick_steering_t;
// 对请求舵角做每秒限速；路径无效时保持旧角度，是否停车由速度规划器依据valid处理。
static float brick_steering(brick_steering_t *s,float requested,int owns,int valid,uint32_t now,float center) {
    if(!s->initialized) {s->angle=center;s->ms=now;s->initialized=1;}
    uint32_t dt=now-s->ms;s->ms=now;
    if(!owns) s->angle=requested;
    else if(valid) {
        if(dt>50u) dt=50u;
        float step=BRICK_STEER_RATE*dt/1000.0f,delta=requested-s->angle;
        if(delta>step) delta=step;
        if(delta< -step) delta= -step;
        s->angle+=delta;
    }
    // 路径无效时仅保持舵角；主循环仍须将速度设为零。
    return s->angle;
}
// 以下小工具只服务红砖检测和安全范围判断。
static int brick_abs(int n) {return n<0?-n:n;}
// 边线坐标需位于图像内部，且两边之间至少保留图像宽度的1/8。
static int brick_pair(int l,int r,int w) {return l>2 && r<w-3 && r-l>=w/8;}

// 仅检测单侧边界向内的台阶，并用真实道路边界约束范围。
// 这是暗色边缘障碍检测器，不按红色色彩分类。
static int brick_detect(const uint8_t *im,const uint8_t *l,const uint8_t *r,
                         const uint8_t *width,const uint8_t *learned,int h,int w,
                         int *top,int *bottom,int *inset)
{
    // 红砖按边线台阶和道路边界几何特征定位，不按颜色分类，降低光照变化的影响。
    int result=0;
    for(int side=1;side<=2;++side) {
        const uint8_t *edge=side==1?l:r,*outer=side==1?r:l;
        int sign=side==1?1:-1;
        for(int a=h/4;a<h*9/10;++a) {
            if(sign*((int)edge[a]-edge[a-1])<w/24 ||
               brick_abs((int)outer[a]-outer[a-1])>2) continue;
            int b=a+1;
            while(b<h-3 && sign*((int)edge[b]-edge[a-1])>=w/24) ++b;
            if(b>=h-3 || b-a<6 || b-a>h/2) continue;
            int ok=1,peak=0,support=0;
            for(int k=0;k<3;++k)
                if(!brick_pair(l[a-1-k],r[a-1-k],w) || !brick_pair(l[b+k],r[b+k],w) ||
                   brick_abs((int)edge[a-1-k]-edge[a-1])>3 ||
                   brick_abs((int)edge[b+k]-edge[b])>3) ok=0;
            for(int y=a;y<b && ok;++y) {
                if(!learned[y] || !brick_pair(l[y],r[y],w)) {ok=0;break;}
                int baseline=edge[a-1]+((int)edge[b]-edge[a-1])*(y-a+1)/(b-a+1);
                int depth=sign*((int)edge[y]-baseline);
                int fit=outer[a-1]+((int)outer[b]-outer[a-1])*(y-a+1)/(b-a+1);
                if(brick_abs((int)outer[y]-fit)>3) {ok=0;break;}
                if(depth>w/24 && width[y]-(r[y]-l[y])>=width[y]/6) {
                    int x=edge[y],dark=0,white=0;
                    for(int k=1;k<=3;++k) {
                        dark+=im[y*w+x-sign*k]==0;
                        white+=im[y*w+x+sign*k]!=0;
                    }
                    if(dark>=2 && white>=2) ++support;
                    if(depth>peak) peak=depth;
                }
            }
            if(!ok || support*5<(b-a)*4) continue;
            if(result && result!=side) return 0;
            result=side; *top=a;*bottom=b-1;*inset=peak;
        }
    }
    return result;
}

static void brick_update(brick_t *s,const uint8_t *im,const uint8_t *l,const uint8_t *r,
                         const uint8_t *width,const uint8_t *learned,int h,int w,
                         uint32_t now,uint32_t distance,int allowed)
{
    // 状态机管理多帧确认、编码器清空距离与恢复；allowed=0 时不启动新的避让接管。
    if(!BRICK_ENABLE || !allowed) {memset(s,0,sizeof *s);return;}
    uint32_t dt=s->initialized?now-s->last_ms:0;
    s->last_ms=now;s->initialized=1;
    if(dt>=BRICK_FRAME_GAP_MS) {
        // 摄像头帧间隔过大时清除图像投票，不清除编码器行程。
        // 已确认的等待/回正起点和避让方向继续保留。
        s->hits=s->clear=s->path_hits=0;
    }
    int top=0,bottom=0,inset=0;
    int candidate=brick_detect(im,l,r,width,learned,h,w,&top,&bottom,&inset);
    s->valid=s->phase!=BRICK_FAULT;
    if(candidate) {
        if(candidate!=s->candidate) s->hits=0;
        if(s->hits<BRICK_CONFIRM_FRAMES) ++s->hits;
        s->candidate=(uint8_t)candidate;
    } else {s->candidate=0;s->hits=0;}
    int confirmed=candidate && s->hits>=BRICK_CONFIRM_FRAMES;
    if(confirmed) {
        if(s->phase!=BRICK_IDLE && candidate!=s->side) {
            // 确认到相反侧的障碍时，不允许中途反转避让方向。
            s->phase=BRICK_FAULT;s->valid=0;s->reason=BRICK_FAILED;
            s->clear=s->path_hits=0;
            return;
        }
        s->top=(uint8_t)top;s->bottom=(uint8_t)bottom;s->inset=(uint8_t)inset;
        s->clear=0;
        if(s->phase==BRICK_IDLE) {
            s->side=(uint8_t)candidate;s->started_ms=now;
        }
        if(s->phase==BRICK_FAULT) s->path_hits=0;
        s->phase=BRICK_AVOID;s->travel=0;s->valid=1;
        float target=inset/2.0f+width[h*4/5]/10.0f;
        if(target>s->target) s->target=target;
    }
    if(s->phase==BRICK_IDLE) return;
    // 首次确认需要连续命中三帧；障碍重新出现也必须满足相同条件。
    // 一两帧噪声不能反复重置清空流程。
    if(!confirmed && s->clear<BRICK_CLEAR_FRAMES) ++s->clear;
    if((s->phase==BRICK_AVOID || s->phase==BRICK_FAULT) &&
       s->clear>=BRICK_CLEAR_FRAMES && !candidate) {
        s->phase=BRICK_HOLD;s->clear_ms=now;s->clear_counts=distance;s->travel=0;
        s->valid=1;
    }
    if(s->phase==BRICK_FAULT) {s->valid=0;s->reason=BRICK_FAILED;return;}
    // 不设置总时长锁死，避免短暂停顿或低速通过后一直占用红砖状态。
    // 仍须满足测得的清空距离，并逐渐回到道路中心。
    if(s->phase==BRICK_HOLD) {
        int32_t forward=(int32_t)(distance-s->clear_counts);
        s->travel=forward>0?(uint32_t)forward:0;
        if(s->travel>=BRICK_CLEAR_COUNTS && now-s->clear_ms>=BRICK_CLEAR_MIN_MS)
            s->phase=BRICK_RECOVER;
    }
    if(dt>50u) dt=50u;
    float desired=s->phase==BRICK_RECOVER?0:s->target;
    float step=BRICK_SHIFT_RATE*dt/1000.0f;
    if(s->offset<desired) {s->offset+=step;if(s->offset>desired)s->offset=desired;}
    else {s->offset-=step;if(s->offset<desired)s->offset=desired;}
    if(s->phase==BRICK_RECOVER && s->offset<=0) {
        memset(s,0,sizeof *s);s->last_ms=now;s->initialized=1;
    }
}

// 单帧候选不代表已开始避让；确认后才接管导航和状态显示，直至清空并回正。
static int brick_owns(const brick_t *s) {return s->phase!=BRICK_IDLE;}
static const char *brick_name(const brick_t *s) {
    if(s->phase==BRICK_FAULT) return "FAULT";
    if(s->phase==BRICK_HOLD) return "HOLD ";
    if(s->phase==BRICK_RECOVER) return "BACK ";
    return s->phase==BRICK_AVOID?"BYPAS":"CHECK";
}
// 只从当前帧恢复边界，绝不复用上一帧的边线。
// 检查边界是否不是图像外沿哨兵；只使用真实可见边界推算避让通道。
static int brick_edge_visible(int x,int w) {return x>2 && x<w-3;}
static int brick_edge_at(const uint8_t *edge,int y,int h,int w) {
    if(brick_edge_visible(edge[y],w)) return edge[y];
    int a=y-1,b=y+1;
    while(a>=0 && y-a<=6 && !brick_edge_visible(edge[a],w)) --a;
    while(b<h && b-y<=6 && !brick_edge_visible(edge[b],w)) ++b;
    if(a<0 || b>=h || y-a>6 || b-y>6) return -1;
    if(brick_abs((int)edge[b]-edge[a])>3*(b-a)) return -1;
    return edge[a]+((int)edge[b]-edge[a])*(y-a)/(b-a);
}
static void brick_apply(brick_t *s,const uint8_t *im,const uint8_t *l,const uint8_t *r,
                        const uint8_t *width,const uint8_t *learned,uint8_t *mid,
                        uint8_t *valid,int h,int w)
{
    // 按障碍所在侧生成避障中线；近、远控制行均有效并连续确认后才允许继续行驶。
    if(s->phase==BRICK_IDLE) return;
    if(s->phase==BRICK_FAULT) {s->valid=0;s->reason=BRICK_FAILED;return;}
    int near=h*4/5,far=h*3/5;
    s->reason=BRICK_OK;
    for(int y=h/4;y<h;++y) {
        valid[y]=0;
        if(!learned[y]) {if(y==near || y==far)s->reason=BRICK_WIDTH;continue;}
        int le=brick_edge_at(l,y,h,w),ri=brick_edge_at(r,y,h,w);
        int one=le<0 || ri<0;
        if(le<0 && ri<0) {if(y==near || y==far)s->reason=BRICK_EDGE;continue;}
        // 可由任一可见边界确定通道；若可见边界正是红砖所在侧，
        // 则将估计中心进一步移离该边界。
        if(le<0) le=ri-width[y];
        if(ri<0) ri=le+width[y];
        int corridor=ri-le,margin=width[y]/10;
        if(margin<3) margin=3;
        if(corridor*100<width[y]*60) {if(y==near || y==far)s->reason=BRICK_NARROW;continue;}
        int base=s->side==1?ri-width[y]/2:le+width[y]/2;
        int target=base+(s->side==1?1:-1)*(int)(s->offset+0.5f);
        // 推算边界可能超出图像；检查白色窗口时须同时限制在估计通道和图像范围内。
        int low=le+margin+1,high=ri-margin-1;
        if(low<margin) low=margin;
        if(high>w-1-margin) high=w-1-margin;
        if(one && low<=high) {
            if(target<low) target=low;
            if(target>high) target=high;
        }
        if(target<low || target>high) {if(y==near || y==far)s->reason=BRICK_SPACE;continue;}
        int white=1;
        for(int x=target-margin;x<=target+margin;++x)
            if(im[y*w+x]==0) white=0;
        if(!white) {if(y==near || y==far)s->reason=BRICK_BLACK;continue;}
        mid[y]=(uint8_t)target;valid[y]=1;
        if(one && (y==near || y==far) && s->reason==BRICK_OK) s->reason=BRICK_ONE;
    }
    // 红砖避让接管期间，不允许无路径时继续盲行。
    if(valid[near] && valid[far]) {
        if(s->path_hits<BRICK_PATH_CONFIRM_FRAMES) ++s->path_hits;
        s->valid=s->path_hits>=BRICK_PATH_CONFIRM_FRAMES;
        if(!s->valid) s->reason=BRICK_WAIT;
    } else {s->path_hits=0;s->valid=0;}
}
#endif
