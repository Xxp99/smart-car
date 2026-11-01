/* Independent image-region roundabout module. 188x120 binary image, white road.
 * 2x sampling: bounded work/memory, no heap, gyro or former A/B/C detector.
 */
#define RV_W 94
#define RV_H 60
#define RV_N (RV_W*RV_H)
#define RV_CONFIRM 4
#define RV_HOLD_MS 160u
#define RV_TIMEOUT_MS 7000u
#define RV_APPROACH_TIMEOUT_MS 15000u
typedef struct {
    uint8_t visited[RV_N];
    uint16_t queue[RV_N];
    uint8_t lo[RV_H], hi[RV_H];
    uint16_t area;
    uint8_t x0,x1,y0,y1,side,hits,miss,exit_hits,cross_hits,entry_armed;
    uint8_t edge_l[2],edge_r[2],white_l[2],white_r[2],snap[2];
    uint32_t stage_ms, good_ms,entry_odo,entry_progress;
} rv_state_t;
static rv_state_t rv;
void round_v2_reset(void) {
    rv.hits=rv.miss=rv.side=rv.cross_hits=rv.exit_hits=0;
    rv.area=0;rv.good_ms=0;rv.entry_armed=0;rv.entry_progress=0;
}

static int rv_clip(int x,int a,int b) {return x<a?a:(x>b?b:x);}
static int rv_pair(int l,int r,int w) {return l>1 && r<w-2 && r-l>w/8;}
/* Find a usable strip of white road at this row. A missing extracted edge
 * must not discard the visible road, but never guide into the black island.
 */
static int rv_white_target(const uint8_t *im,int y,int w,int wanted,int road_left,
                           int width,int *out_l,int *out_r)
{
    int island_l=-1,island_r=-1,best=-1,best_cost=10000;
    if(rv.area && rv.lo[y/2]!=255) {
        island_l=rv.lo[y/2]*2;island_r=rv.hi[y/2]*2;
    }
    for(int x=2;x<w-2;) {
        if(im[y*w+x]<128){++x;continue;}
        int a=x;
        while(x<w-2 && im[y*w+x]>=128)++x;
        int b=x-1;
        if(b-a+1<10)continue;
        if(island_l>=0 && (road_left?a>=island_l-2:b<=island_r+2))continue;
        int p=rv_clip(wanted,a+3,b-3);
        int cost=abs(p-wanted);
        if(cost<best_cost){best_cost=cost;best=p;*out_l=a;*out_r=b;}
    }
    if(best<0 || best_cost>width/2)return -1;
    return best;
}
track_element_t round_v2_observe(const uint8_t *im,const uint8_t *l,const uint8_t *r,
                                uint8_t h,uint8_t w)
{
    rv.area=0;
    memset(rv.lo,255,sizeof rv.lo); memset(rv.hi,0,sizeof rv.hi);
    if (!im || w!=188 || h!=120) return TRACK_UNKNOWN;
    memset(rv.visited,0,sizeof rv.visited);
    /* An island may be clipped at one image edge. Background spanning image
     * corners/multiple edges and anything touching the bottom are rejected.
     */
    for(int seed=0;seed<RV_N;++seed) {
        int sx=seed%RV_W, sy=seed/RV_W;
        if(rv.visited[seed] || im[(sy*2)*w+sx*2]>127) continue;
        int head=0,tail=1,x0=sx,x1=sx,y0=sy,y1=sy,border=0;
        rv.queue[0]=(uint16_t)seed; rv.visited[seed]=1;
        while(head<tail) {
            int p=rv.queue[head++],x=p%RV_W,y=p/RV_W;
            if(x<x0)x0=x; if(x>x1)x1=x; if(y<y0)y0=y; if(y>y1)y1=y;
            if(x==0)border|=1;
            if(x==RV_W-1)border|=2;
            if(y==0)border|=4;
            if(y==RV_H-1)border|=8;
            const int dx[4]={-1,1,0,0},dy[4]={0,0,-1,1};
            for(int k=0;k<4;++k) {
                int xx=x+dx[k],yy=y+dy[k];
                if(xx<0 || xx>=RV_W || yy<0 || yy>=RV_H)continue;
                int q=yy*RV_W+xx;
                if(!rv.visited[q] && im[(yy*2)*w+xx*2]<=127) {
                    rv.visited[q]=1; rv.queue[tail++]=(uint16_t)q;
                }
            }
        }
        int bw=x1-x0+1,bh=y1-y0+1;
        if((border&8) || (border && (border&(border-1))) ||
           tail<100 || bw<14 || bh<6 || bw>70 || bh>55 ||
           bw>bh*5 || tail<bw*bh/3 || tail>bw*bh*9/10 || y1<10 || y0>45)continue;
        if(tail>rv.area) {
            rv.area=(uint16_t)tail;rv.x0=(uint8_t)(2*x0);rv.x1=(uint8_t)(2*x1);
            rv.y0=(uint8_t)(2*y0);rv.y1=(uint8_t)(2*y1);
            memset(rv.lo,255,sizeof rv.lo);memset(rv.hi,0,sizeof rv.hi);
            for(int j=0;j<tail;++j) {
                int x=rv.queue[j]%RV_W,y=rv.queue[j]/RV_W;
                if(x<rv.lo[y])rv.lo[y]=(uint8_t)x;
                if(x>rv.hi[y])rv.hi[y]=(uint8_t)x;
            }
        }
    }
    int left_gap=0,right_gap=0,left_ok=0,right_ok=0;
    for(int y=h/3;y<h*9/10;++y) {
        left_gap+=l[y]<=1;right_gap+=r[y]>=w-2;
        left_ok+=l[y]>1 && l[y]<w-2;
        right_ok+=r[y]>1 && r[y]<w-2;
    }
    if(rv.area) {
        int cx=(rv.x0+rv.x1)/2;
        if(cx<w/2 && left_gap>=5 && right_ok>=h/4) return TRACK_ROUND_LEFT;
        if(cx>w/2 && right_gap>=5 && left_ok>=h/4) return TRACK_ROUND_RIGHT;
    }
    int gap=0;
    for(int y=h/3;y<h-2;++y) {
        if(l[y]<=1 && r[y]>=w-2)++gap;
        else if(gap) {
            int top=y-gap-1;
            if(gap>=6 && gap<h/3 && top>=0 && rv_pair(l[top],r[top],w) &&
               rv_pair(l[y],r[y],w))return TRACK_CROSS;
            gap=0;
        }
    }
    return rv_pair(l[h*4/5],r[h*4/5],w)?TRACK_NONE:TRACK_UNKNOWN;
}

void round_v2_detector(track_detector_t *d,track_element_t kind)
{
    uint8_t side=kind==TRACK_ROUND_LEFT?1:(kind==TRACK_ROUND_RIGHT?2:0);
    if(side && side==rv.side) {if(rv.hits<6)++rv.hits;rv.miss=0;}
    else if(side) {rv.side=side;rv.hits=1;rv.miss=0;}
    else {if(rv.miss<3)++rv.miss;if(rv.miss>=3){rv.hits=0;rv.side=0;} }
    d->candidate=kind;d->ring_votes=rv.hits;
    d->confirmed=side && rv.hits>=RV_CONFIRM?kind:TRACK_NONE;
    if(kind==TRACK_CROSS){if(rv.cross_hits<4)++rv.cross_hits;}
    else rv.cross_hits=0;
    if(rv.cross_hits>=4)d->confirmed=TRACK_CROSS;
}

void round_v2_navigate(track_nav_t *n,const track_detector_t *d,
    const uint8_t *raw_l,const uint8_t *raw_r,const uint8_t *im,
    uint8_t *l,uint8_t *r,uint8_t *mid,uint8_t h,uint8_t w,uint32_t now,uint32_t odo)
{
    int near=h*4/5,far=h*3/5;
    n->vision.enabled=0;n->entry_guide=0;
    if(n->phase==NAV_NORMAL && d->confirmed!=TRACK_NONE) {
        n->element=d->confirmed;n->phase=d->confirmed==TRACK_CROSS?NAV_CROSS:NAV_APPROACH;n->started_ms=now;
        rv.stage_ms=now;rv.exit_hits=0;rv.entry_armed=0;
        rv.entry_progress=0;n->fault=NAV_FAULT_NONE;
    }
    int active=n->phase!=NAV_NORMAL;
    int pending=d->candidate==TRACK_ROUND_LEFT || d->candidate==TRACK_ROUND_RIGHT;
    int left=(active?n->element:d->candidate)==TRACK_ROUND_LEFT;
    if(active && now-rv.stage_ms>
       (n->phase==NAV_APPROACH?RV_APPROACH_TIMEOUT_MS:RV_TIMEOUT_MS)) {
        n->phase=NAV_SLOW;n->fault=NAV_FAULT_TIMEOUT;
    }
    /* Phase changes require the tracked island, never elapsed time alone. */
    if(n->phase==NAV_APPROACH && rv.area && rv.y1>=h/2 && !rv.entry_armed) {
        rv.entry_armed=1;rv.entry_odo=odo;
    }
    if(rv.entry_armed) {
        int32_t delta=(int32_t)(odo-rv.entry_odo);
        rv.entry_progress=delta>0?(uint32_t)delta:0u;
    }
    if(n->phase==NAV_APPROACH && rv.entry_armed &&
       (!left || rv.entry_progress>=ROUND_V2_A_TO_C_COUNTS)) {
        n->phase=NAV_ENTER;rv.stage_ms=now;
    } else if(n->phase==NAV_ENTER && rv.area && rv.y1>=h*4/5) {
        n->phase=NAV_INSIDE;rv.stage_ms=now;
    }
    for(int y=0;y<h;++y) {
        l[y]=raw_l[y];r[y]=raw_r[y];
        int width=(int)r[y]-l[y],valid=rv_pair(l[y],r[y],w);
        if(!active && valid && width<w*4/5) {
            n->width[y]=n->learned[y]?(7*n->width[y]+width)/8:width;
            n->learned[y]=1;
        }
        int center=((int)l[y]+r[y])/2;
        if(n->phase==NAV_CROSS && !valid) {
            int a=y-1,b=y+1;
            while(a>0 && !rv_pair(raw_l[a],raw_r[a],w))--a;
            while(b<h-1 && !rv_pair(raw_l[b],raw_r[b],w))++b;
            if(b-a<h/2 && rv_pair(raw_l[a],raw_r[a],w) && rv_pair(raw_l[b],raw_r[b],w)) {
                int ca=(raw_l[a]+raw_r[a])/2,cb=(raw_l[b]+raw_r[b])/2;
                center=ca+(cb-ca)*(y-a)/(b-a);valid=1;
            }
        }
        if(n->learned[y] && (active || pending) && n->phase!=NAV_CROSS) {
            width=n->width[y];
            int outer=left?r[y]:l[y];
            int island_row=rv.area && rv.lo[y/2]!=255;
            /* As the camera rotates around the island, its image crosses the
             * centre. The drivable corridor then lies on the other image side.
             * Ring direction alone cannot select the side of the black blob.
             */
            int road_left=island_row?((rv.lo[y/2]+rv.hi[y/2])>=RV_W):!left;
            if(active && !valid && n->last_mid[y])center=n->last_mid[y];
            if(outer>1 && outer<w-2) {
                center=left?outer-width/2:outer+width/2;valid=1;
                if(n->phase==NAV_ENTER) center+=(left?-1:1)*width/4;
            }
            if(n->phase==NAV_APPROACH && left && rv.entry_armed) {
                center=rv_pair(raw_l[y],raw_r[y],w)?
                    ((int)raw_l[y]+raw_r[y])/2:
                    (n->last_mid[y]?n->last_mid[y]:w/2);
                valid=1;
            }
            if(n->phase==NAV_ENTER && island_row && road_left) {
                center=rv.lo[y/2]*2-width/2;valid=1;
            }
            if((n->phase==NAV_INSIDE || n->phase==NAV_EXIT) && island_row) {
                center=road_left?rv.lo[y/2]*2-width/2:rv.hi[y/2]*2+width/2;
                valid=1;
            }
            if(n->phase==NAV_EXIT && !island_row &&
               !rv_pair(raw_l[y],raw_r[y],w)) {
                center=w/2;valid=1;
            }
            if(active) {
                int a=0,b=0;
                int wanted=rv_clip(center,3,w-4);
                int safe=rv_white_target(im,y,w,wanted,road_left,width,&a,&b);
                if(safe>=0){center=safe;valid=1;}
                else valid=0;
                if(y==near || y==far){
                    int k=y==near?0:1;
                    rv.edge_l[k]=raw_l[y];rv.edge_r[k]=raw_r[y];
                    rv.white_l[k]=safe>=0?a:0;rv.white_r[k]=safe>=0?b:0;
                    rv.snap[k]=safe>=0?(uint8_t)abs(safe-wanted):255;
                }
            }
        }
        if(valid && (center<3 || center>w-4 || im[y*w+rv_clip(center,0,w-1)]<128))valid=0;
        n->path_valid[y]=(uint8_t)valid;
        if(valid) {
            mid[y]=(uint8_t)center;n->last_mid[y]=mid[y];
        } else mid[y]=n->last_mid[y]?n->last_mid[y]:w/2;
    }
    int good=n->path_valid[near] && n->path_valid[far];
    if(good) {rv.good_ms=now;n->last_path_ms=now;}
    n->drive_valid=good || (active && rv.good_ms && now-rv.good_ms<RV_HOLD_MS);
    if(active && !n->drive_valid)n->fault=NAV_FAULT_PATH;
    else if(n->fault==NAV_FAULT_PATH)n->fault=NAV_FAULT_NONE;
    if(n->phase==NAV_INSIDE && now-rv.stage_ms>600u && !rv.area && good &&
       abs((int)mid[near]-mid[far])<w/16) {
        if(++rv.exit_hits>=6){n->phase=NAV_EXIT;rv.stage_ms=now;rv.exit_hits=0;}
    } else if(n->phase==NAV_INSIDE)rv.exit_hits=0;
    /* The synthetic white path can be valid while the camera still sees only
     * one real edge. Do not return to normal tracking until both real edges
     * have reappeared at the two steering rows. */
    if(n->phase==NAV_EXIT && good && now-rv.stage_ms>400u &&
       rv_pair(raw_l[near],raw_r[near],w) &&
       rv_pair(raw_l[far],raw_r[far],w)) {
        n->phase=NAV_NORMAL;n->element=TRACK_NONE;n->fault=NAV_FAULT_NONE;
        rv.hits=0;rv.side=0;
    }
    if(n->phase==NAV_CROSS && good && d->candidate==TRACK_NONE && now-rv.stage_ms>400u) {
        n->phase=NAV_NORMAL;n->element=TRACK_NONE;n->fault=NAV_FAULT_NONE;
    }
    if(n->phase==NAV_SLOW)n->drive_valid=0;
    n->entry_guide=n->phase==NAV_ENTER;
    n->apex_valid=rv.area!=0;n->apex_x=(rv.x0+rv.x1)/2;n->apex_y=rv.y1;
    n->stage_ms=rv.stage_ms;
}

void round_v2_diagnostics(char *out,uint32_t length,float servo)
{
    snprintf(out,length," V2 area=%u box=%u,%u,%u,%u side=%u hits=%u gate=%lu/%u servo=%.1f edge=%u,%u/%u,%u white=%u-%u/%u-%u snap=%u,%u",
        (unsigned)rv.area,(unsigned)rv.x0,(unsigned)rv.y0,(unsigned)rv.x1,(unsigned)rv.y1,
        (unsigned)rv.side,(unsigned)rv.hits,(unsigned long)rv.entry_progress,
        (unsigned)ROUND_V2_A_TO_C_COUNTS,(double)servo,
        (unsigned)rv.edge_l[0],(unsigned)rv.edge_r[0],(unsigned)rv.edge_l[1],(unsigned)rv.edge_r[1],
        (unsigned)rv.white_l[0],(unsigned)rv.white_r[0],(unsigned)rv.white_l[1],(unsigned)rv.white_r[1],
        (unsigned)rv.snap[0],(unsigned)rv.snap[1]);
}
