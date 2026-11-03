// 红砖避让回归测试：用合成边线验证双侧检测、连续确认、偏移方向、保持与编码器清空距离。
#include <assert.h>
#include <stdio.h>
#include "../镜头组例程/demo/user/inc/brick_avoid.h"
static uint8_t im[120*188],l[120],r[120],width[120],learned[120],mid[120],valid[120];
static void scene(int side,int top,int bottom,int depth) {
    memset(im,0,sizeof im);
    for(int y=0;y<120;++y) {
        l[y]=50;r[y]=138;width[y]=88;learned[y]=1;mid[y]=94;valid[y]=1;
        if(y>=top && y<=bottom) {if(side==1)l[y]+=depth;if(side==2)r[y]-=depth;}
        for(int x=l[y]+1;x<r[y];++x) im[y*188+x]=255;
    }
}
static void tick(brick_t *s,uint32_t t,uint32_t d) {
    brick_update(s,im,l,r,width,learned,120,188,t,d,1);
    brick_apply(s,im,l,r,width,learned,mid,valid,120,188);
}
int main(void) {
    int a,b,depth;
    scene(0,0,0,0);assert(!brick_detect(im,l,r,width,learned,120,188,&a,&b,&depth));
    scene(1,50,53,30);assert(!brick_detect(im,l,r,width,learned,120,188,&a,&b,&depth));
    for(int side=1;side<=2;++side) {
        brick_t s={0};scene(side,50,75,30);
        assert(brick_detect(im,l,r,width,learned,120,188,&a,&b,&depth)==side);
        tick(&s,0,0);tick(&s,50,100);assert(s.phase==BRICK_IDLE);
        for(int f=2;f<14;++f) tick(&s,50*f,100*f);
        assert(s.phase==BRICK_AVOID && s.valid);
        assert(side==1?mid[96]>94:mid[96]<94);
        float held=s.offset;
        scene(0,0,0,0);
        for(int f=14;f<20;++f) tick(&s,50*f,2000);
        assert(s.phase==BRICK_HOLD && s.offset==held);
        for(int f=20;f<50;++f) tick(&s,50*f,2000);
        assert(s.phase==BRICK_HOLD && s.travel==0); // Time alone never releases.
        tick(&s,2500,1900);assert(s.travel==0); // Reverse is not forward clearance.
        tick(&s,2550,2000+BRICK_CLEAR_COUNTS-1);assert(s.phase==BRICK_HOLD);
        tick(&s,2600,2000+BRICK_CLEAR_COUNTS);assert(s.phase==BRICK_RECOVER);
        float previous=s.offset;
        for(int f=53;f<70;++f) {
            tick(&s,f*50,2000+BRICK_CLEAR_COUNTS);assert(s.offset<=previous);previous=s.offset;
        }
        assert(s.phase==BRICK_IDLE && !brick_owns(&s));
        // Reappearance cancels clearance; opposite side cannot reverse the manoeuvre.
        memset(&s,0,sizeof s);scene(side,50,75,30);
        for(int f=0;f<12;++f) tick(&s,50*f,100*f);
        scene(0,0,0,0);for(int f=12;f<18;++f)tick(&s,50*f,2000);
        assert(s.phase==BRICK_HOLD);
        scene(side,50,75,30);tick(&s,900,3000);assert(s.phase==BRICK_AVOID && s.travel==0);
        scene(side==1?2:1,50,75,30);tick(&s,950,3100);
        assert(s.phase==BRICK_FAULT && !s.valid);
        tick(&s,1500,4000);assert(s.phase==BRICK_FAULT); // A camera gap cannot clear a fault.
    }
    // Reaching the distance early cannot bypass the minimum holding time.
    brick_t fast={0};scene(1,50,75,30);
    for(int f=0;f<12;++f)tick(&fast,50*f,0);
    scene(0,0,0,0);for(int f=12;f<18;++f)tick(&fast,50*f,2000);
    assert(fast.phase==BRICK_HOLD);
    for(uint32_t t=900;t<fast.clear_ms+BRICK_CLEAR_MIN_MS;t+=50) {
        tick(&fast,t,2000+BRICK_CLEAR_COUNTS);
        assert(fast.phase==BRICK_HOLD);
    }
    tick(&fast,fast.clear_ms+BRICK_CLEAR_MIN_MS,2000+BRICK_CLEAR_COUNTS);
    assert(fast.phase==BRICK_RECOVER);
    // Gap invalidates distance progress; wrap-around remains supported.
    brick_t s={0};scene(1,50,75,30);
    for(int f=0;f<12;++f)tick(&s,50*f,0);
    scene(0,0,0,0);for(int f=12;f<18;++f)tick(&s,50*f,UINT32_MAX-100);
    assert(s.phase==BRICK_HOLD);
    tick(&s,1200,6000);assert(s.phase==BRICK_AVOID && !s.travel);
    for(int f=25;f<30;++f)tick(&s,50*f,UINT32_MAX-100);
    for(int f=30;f<44;++f)tick(&s,50*f,UINT32_MAX-100);
    tick(&s,2200,(UINT32_MAX-100)+BRICK_CLEAR_COUNTS);
    assert(s.phase==BRICK_RECOVER);
    // Path is too narrow: no blind hold of an unsafe center.
    scene(1,80,105,50);brick_apply(&s,im,l,r,width,learned,mid,valid,120,188);
    assert(!s.valid);
    tick(&s,9000,0);assert(s.phase==BRICK_FAULT);
    // No learned width => no new obstacle claim; open edges/crosses are not bricks.
    scene(1,50,75,30);memset(learned,0,sizeof learned);
    assert(!brick_detect(im,l,r,width,learned,120,188,&a,&b,&depth));
    scene(0,0,0,0);for(int y=50;y<=75;++y){l[y]=0;r[y]=187;}
    assert(!brick_detect(im,l,r,width,learned,120,188,&a,&b,&depth));
    // Curves move both sides together.
    scene(0,0,0,0);for(int y=50;y<=75;++y){l[y]+=25;r[y]+=25;}
    assert(!brick_detect(im,l,r,width,learned,120,188,&a,&b,&depth));
    puts("PASS: mirrored edge obstacles, geometry rejection, direction lock, distance+time clearance, reverse, camera gaps, wrap, recovery, unsafe corridor, fault latch");
}
