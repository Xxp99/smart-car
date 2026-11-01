#ifndef BRICK_AVOID_H
#define BRICK_AVOID_H
#include <stdint.h>
#include <string.h>

#ifndef BRICK_ENABLE
#define BRICK_ENABLE 1
#endif
#define BRICK_SPEED 45.0f
#define BRICK_CONFIRM_FRAMES 3
#define BRICK_CLEAR_FRAMES 5
// Measured forward odometer delta over 1 metre (2026-11-01).
#define BRICK_COUNTS_PER_METER 11000u
// Include the brick length in case its visual feature disappears early.
// 33 cm is a provisional blind-zone/margin budget, NOT a measured blind zone.
// Encoder travel is not exact longitudinal clearance when turning or slipping.
#define BRICK_CAMERA_TO_REAR_CM 22u
#define BRICK_LENGTH_CM 27u
#define BRICK_BLIND_MARGIN_CM 33u
#define BRICK_CLEAR_DISTANCE_CM (BRICK_CAMERA_TO_REAR_CM + BRICK_LENGTH_CM + BRICK_BLIND_MARGIN_CM)
#define BRICK_CLEAR_COUNTS ((BRICK_COUNTS_PER_METER * BRICK_CLEAR_DISTANCE_CM + 99u) / 100u)
#define BRICK_CLEAR_MIN_MS 600u
#define BRICK_TIMEOUT_MS 8000u
#define BRICK_FRAME_GAP_MS 300u
#define BRICK_SHIFT_RATE 60.0f // image pixels per second

typedef enum {BRICK_IDLE,BRICK_AVOID,BRICK_HOLD,BRICK_RECOVER,BRICK_FAULT} brick_phase_t;
typedef struct {
    brick_phase_t phase;
    uint8_t side,candidate,hits,clear,valid,initialized;
    uint8_t top,bottom,inset;
    uint32_t started_ms,last_ms,clear_ms,clear_counts,travel;
    float offset,target;
} brick_t;
static int brick_abs(int n) {return n<0?-n:n;}
static int brick_pair(int l,int r,int w) {return l>2 && r<w-3 && r-l>=w/8;}

// Local INWARD step on only one border, bounded above/below by actual road.
// This is a dark edge obstacle detector, not a red-colour classifier.
static int brick_detect(const uint8_t *im,const uint8_t *l,const uint8_t *r,
                         const uint8_t *width,const uint8_t *learned,int h,int w,
                         int *top,int *bottom,int *inset)
{
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
    if(!BRICK_ENABLE || !allowed) {memset(s,0,sizeof *s);return;}
    uint32_t dt=s->initialized?now-s->last_ms:0;
    s->last_ms=now;s->initialized=1;
    if(s->phase==BRICK_FAULT) {s->valid=0;return;}
    if(dt>=BRICK_FRAME_GAP_MS) {
        s->hits=s->clear=0;
        if(s->phase!=BRICK_IDLE) {
            // After a camera gap, retain the side but rebuild the clearance distance.
            s->phase=BRICK_AVOID;s->clear_counts=distance;s->travel=0;
        }
    }
    int top=0,bottom=0,inset=0;
    int candidate=brick_detect(im,l,r,width,learned,h,w,&top,&bottom,&inset);
    s->valid=1;
    if(candidate && s->phase!=BRICK_IDLE && candidate!=s->side) {
        s->phase=BRICK_FAULT;s->valid=0;return; // Never alternate sides mid-pass.
    }
    if(candidate) {
        if(candidate!=s->candidate) s->hits=0;
        if(s->hits<BRICK_CONFIRM_FRAMES) ++s->hits;
        s->candidate=(uint8_t)candidate;s->top=(uint8_t)top;
        s->bottom=(uint8_t)bottom;s->inset=(uint8_t)inset;s->clear=0;
        if(s->phase==BRICK_IDLE && s->hits>=BRICK_CONFIRM_FRAMES) {
            s->side=(uint8_t)candidate;s->phase=BRICK_AVOID;s->started_ms=now;
        }
        if(s->phase!=BRICK_IDLE) {
            s->phase=BRICK_AVOID;s->travel=0;
            float target=inset/2.0f+width[h*4/5]/10.0f;
            if(target>s->target) s->target=target;
        }
    } else {s->candidate=0;s->hits=0;}
    if(s->phase==BRICK_IDLE) return;
    if(now-s->started_ms>=BRICK_TIMEOUT_MS) {s->phase=BRICK_FAULT;s->valid=0;return;}
    // Losing the object is not evidence that the body has passed it.
    int road_clear=!candidate;
    for(int y=h/3;y<h-3 && road_clear;++y)
        if(!learned[y] || !brick_pair(l[y],r[y],w) ||
           brick_abs((r[y]-l[y])-(int)width[y])>width[y]/8) road_clear=0;
    if(road_clear) {if(s->clear<BRICK_CLEAR_FRAMES) ++s->clear;}
    else {s->clear=0; if(s->phase==BRICK_HOLD || s->phase==BRICK_RECOVER) s->phase=BRICK_AVOID;}
    if(s->phase==BRICK_AVOID && s->clear>=BRICK_CLEAR_FRAMES) {
        s->phase=BRICK_HOLD;s->clear_ms=now;s->clear_counts=distance;s->travel=0;
    }
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

static int brick_owns(const brick_t *s) {return s->candidate || s->phase!=BRICK_IDLE;}
static const char *brick_name(const brick_t *s) {
    if(s->phase==BRICK_FAULT) return "FAULT";
    if(s->phase==BRICK_HOLD) return "HOLD ";
    if(s->phase==BRICK_RECOVER) return "BACK ";
    return s->phase==BRICK_AVOID?"BYPAS":"CHECK";
}
static void brick_apply(brick_t *s,const uint8_t *im,const uint8_t *l,const uint8_t *r,
                        const uint8_t *width,const uint8_t *learned,uint8_t *mid,
                        uint8_t *valid,int h,int w)
{
    if(s->phase==BRICK_IDLE) return;
    if(s->phase==BRICK_FAULT) {s->valid=0;return;}
    int near=h*4/5,far=h*3/5;
    for(int y=h/4;y<h;++y) {
        valid[y]=0;
        if(!learned[y] || !brick_pair(l[y],r[y],w)) continue;
        int corridor=r[y]-l[y],margin=width[y]/10;
        if(margin<3) margin=3;
        if(corridor*100<width[y]*60) continue;
        int base=s->side==1?r[y]-width[y]/2:l[y]+width[y]/2;
        int target=base+(s->side==1?1:-1)*(int)(s->offset+0.5f);
        if(target<l[y]+margin || target>r[y]-margin) continue;
        int white=1;
        for(int x=target-margin;x<=target+margin;++x)
            if(im[y*w+x]==0) white=0;
        if(!white) continue;
        mid[y]=(uint8_t)target;valid[y]=1;
    }
    // No blind-path grace while an obstacle owns steering.
    s->valid=valid[near] && valid[far];
}
#endif
