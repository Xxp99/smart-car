"""Compile only the new module: no former ring detector or navigation linked."""
from pathlib import Path
import subprocess, tempfile, shutil
ROOT=Path(__file__).resolve().parents[1]
USER=ROOT/'镜头组例程/demo/user'
HARNESS=r'''
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "img_process.h"
#include "roundabout_v2.inc"
static uint8_t im[120*188], l[120],r[120],cl[120],cr[120],mid[120];
static void scene(int side,int island,int cy) {
    for(int y=0;y<120;++y) {
        l[y]=25;r[y]=163;
        for(int x=0;x<188;++x) {
            int xx=side==2?187-x:x;
            int black=island && (xx-55)*(xx-55)*64+(y-cy)*(y-cy)*400<=25600;
            im[y*188+x]=x>25 && x<163 && !black?255:0;
        }
        if(side && y>20 && y<80) {if(side==1)l[y]=0;else r[y]=187;}
    }
}
int main(void) {
    for(int side=1;side<=2;++side) {
        memset(&rv,0,sizeof rv);
        track_nav_t n={0};track_detector_t d={0};
        scene(0,0,40);
        for(unsigned t=0;t<200;t+=40) {
            round_v2_detector(&d,round_v2_observe(im,l,r,120,188));
            round_v2_navigate(&n,&d,l,r,im,cl,cr,mid,120,188,t,t*40);
            assert(d.confirmed==TRACK_NONE && n.phase==NAV_NORMAL);
        }
        scene(side,0,40);
        assert(round_v2_observe(im,l,r,120,188)!=TRACK_ROUND_LEFT);
        assert(round_v2_observe(im,l,r,120,188)!=TRACK_ROUND_RIGHT);
        scene(side,0,40);
        for(int y=0;y<120;++y)for(int x=0;x<188;++x) {
            int xx=side==1?x:187-x;
            if((xx-55)*(xx-55)*400+(y-8)*(y-8)*784<=313600)
                im[y*188+x]=0;
        }
        assert(round_v2_observe(im,l,r,120,188)==(side==1?TRACK_ROUND_LEFT:TRACK_ROUND_RIGHT));
        scene(side,0,40);
        for(int y=45;y<=70;++y)for(int x=35;x<=75;++x)
            im[y*188+(side==1?x:187-x)]=0;
        assert(round_v2_observe(im,l,r,120,188)!=TRACK_ROUND_LEFT);
        assert(round_v2_observe(im,l,r,120,188)!=TRACK_ROUND_RIGHT);
        scene(side,1,60);
        for(unsigned t=200;t<800;t+=40) {
            track_element_t c=round_v2_observe(im,l,r,120,188);
            assert(c==(side==1?TRACK_ROUND_LEFT:TRACK_ROUND_RIGHT));
            round_v2_detector(&d,c);
            round_v2_navigate(&n,&d,l,r,im,cl,cr,mid,120,188,t,t*40);
            if(side==1 && t==600)assert(n.phase==NAV_APPROACH);
        }
        assert(n.phase==NAV_ENTER && n.drive_valid);
        assert(side==1?mid[96]<94:mid[96]>94);
        /* Both extracted edges can vanish at the entry while white road is
         * still visible beside the island. This was the real path=00 stop. */
        if(side==1) {
            for(int y=0;y<120;++y) {
                l[y]=0;r[y]=187;
                for(int x=0;x<188;++x)
                    im[y*188+x]=(x>=20 && x<=170)?255:0;
            }
            for(int x=65;x<=150;++x)im[72*188+x]=0;
            for(int x=80;x<=110;++x)im[96*188+x]=0;
            rv.area=725;rv.y1=82;
            memset(rv.lo,255,sizeof rv.lo);
            rv.lo[36]=32;rv.hi[36]=75;
            n.last_mid[96]=84;n.last_mid[72]=93;
            round_v2_navigate(&n,&d,l,r,im,cl,cr,mid,120,188,840,33600);
            assert(n.path_valid[96] && n.path_valid[72] && n.drive_valid);
            assert(mid[96]<80 && mid[72]<65);
        }
        scene(side,1,90);
        round_v2_observe(im,l,r,120,188);
        round_v2_navigate(&n,&d,l,r,im,cl,cr,mid,120,188,880,35200);
        assert(n.phase==NAV_INSIDE);
        if(side==1) {
            /* w.2: island moved to the right edge; the former formula aimed
             * past x=187 and rejected the visible white road at x=9..108. */
            for(int y=0;y<120;++y) {
                l[y]=0;r[y]=187;
                for(int x=0;x<188;++x)im[y*188+x]=(x>=2 && x<=185)?255:0;
            }
            for(int x=109;x<188;++x)im[72*188+x]=0;
            for(int x=137;x<188;++x)im[96*188+x]=0;
            l[72]=8;r[72]=109;l[96]=0;r[96]=137;
            memset(rv.lo,255,sizeof rv.lo);
            rv.lo[36]=54;rv.hi[36]=93;
            rv.lo[48]=70;rv.hi[48]=93;
            rv.area=953;rv.x0=108;rv.x1=186;rv.y0=48;rv.y1=108;
            round_v2_navigate(&n,&d,l,r,im,cl,cr,mid,120,188,920,36800);
            assert(n.path_valid[96] && n.path_valid[72] && n.drive_valid);
            assert(mid[96]<94 && mid[72]<94);
        }
        memset(im,0,sizeof im);memset(l,0,sizeof l);memset(r,187,sizeof r);
        round_v2_observe(im,l,r,120,188);
        round_v2_navigate(&n,&d,l,r,im,cl,cr,mid,120,188,1320,36800);
        assert(!n.drive_valid);
        assert(round_v2_observe(im,l,r,120,188)==TRACK_UNKNOWN);
    }
    scene(0,0,40);
    for(int y=50;y<60;++y){l[y]=0;r[y]=187;}
    assert(round_v2_observe(im,l,r,120,188)==TRACK_CROSS);
    {
        track_nav_t n={0};track_detector_t d={0};
        n.phase=NAV_EXIT;n.element=TRACK_ROUND_LEFT;
        n.learned[96]=n.learned[72]=1;
        n.width[96]=129;n.width[72]=97;
        n.last_mid[96]=100;n.last_mid[72]=110;
        rv.area=0;rv.stage_ms=0;
        for(int y=0;y<120;++y) {
            l[y]=20;r[y]=187;
            for(int x=0;x<188;++x)im[y*188+x]=(x>20 && x<186)?255:0;
        }
        round_v2_navigate(&n,&d,l,r,im,cl,cr,mid,120,188,600,0);
        assert(n.phase==NAV_EXIT && n.drive_valid);
        assert(mid[96]==94 && mid[72]==94);
        for(int y=0;y<120;++y)r[y]=160;
        round_v2_navigate(&n,&d,l,r,im,cl,cr,mid,120,188,700,0);
        assert(n.phase==NAV_NORMAL);
    }
    puts("PASS: independent region detector, mirrored entry, island progression, loss stop");
}
'''
def main():
    with tempfile.TemporaryDirectory(prefix='round_v2_') as tmp:
        p=Path(tmp)
        header=(USER/'inc/img_process.h').read_text(encoding='utf-8')
        (p/'img_process.h').write_text(header.replace('#include "zf_common_headfile.h"','#include <stdint.h>'),encoding='utf-8')
        (p/'test.c').write_text(HARNESS,encoding='utf-8')
        subprocess.run([shutil.which('cl'),'/nologo','/std:c11','/utf-8','/W4',f'/I{p}',f'/I{USER / "src"}',str(p/'test.c'),f'/Fe:{p / "test.exe"}'],cwd=p,check=True)
        subprocess.run([str(p/'test.exe')],check=True)
if __name__=='__main__':main()
