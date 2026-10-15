#include "position_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static navigation_snapshot_t valid_pose(float yaw)
{
    navigation_snapshot_t p={0};p.valid=p.bias_ready=true;p.status=NAV_RUNNING;p.yaw_deg=yaw;return p;
}
int main(void)
{
    position_control_t c;
    position_config_t cfg=POSITION_CONFIG_DEFAULT;
    position_goal_t goal={0,0,0};
    navigation_snapshot_t p=valid_pose(15);
    position_control_reset(&c);
    for(unsigned i=0;i<50;++i) {
        position_control_update(&c,&cfg,&goal,&p,.01f);
        assert(c.output.status!=POSITION_REACHED && c.output.omega_radps<0);
    }
    for(unsigned i=0;i<200 && c.output.status>=0;++i)
        position_control_update(&c,&cfg,&goal,&p,.01f);
    assert(c.output.status==POSITION_YAW_STALLED && c.output.omega_radps==0);
    position_control_update(&c,&cfg,&goal,&p,.01f);assert(c.output.status==POSITION_YAW_STALLED);
    for(int sign=-1;sign<=1;sign+=2) {
        p=valid_pose(sign*15);position_control_reset(&c);
        cfg.yaw_tolerance_deg=1;
        unsigned i;
        for(i=0;i<2000;++i) {
            position_control_update(&c,&cfg,&goal,&p,.01f);
            if(c.output.status==POSITION_REACHED) break;
            assert(c.output.status>=0);
            p.yaw_deg+=c.output.omega_radps*.01f*57.2957795f;
        }
        assert(i<2000 && fabsf(p.yaw_deg)<=1);
    }
    position_control_reset(&c);p=valid_pose(0);
    for(unsigned i=0;i<25;++i) position_control_update(&c,&cfg,&goal,&p,.01f);
    assert(c.output.status==POSITION_REACHED);
    puts("yaw arrival passed: no premature success at 15deg, blocked rotation faults, both-direction alignment, zero-angle settling");
}
