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
    /* Regression: +1.78 degrees must not be accepted just because TolYaw=3.
     * Heading must also be corrected while moving, and crawl stall must not
     * bypass heading alignment. A stuck small offset must fault, not succeed. */
    cfg=(position_config_t)POSITION_CONFIG_DEFAULT;
    for(int sign=-1;sign<=1;sign+=2) {
        position_goal_t moving_goal={300,0,0};
        p=valid_pose(sign*1.78f); position_control_reset(&c);
        for(unsigned j=0;j<30;++j) {
            position_control_update(&c,&cfg,&moving_goal,&p,.01f);
            assert(sign*c.output.omega_radps<0);
            assert(c.output.status==POSITION_MOVING);
        }
        p=valid_pose(sign*1.78f); position_control_reset(&c);
        for(unsigned j=0;j<60;++j) {
            position_control_update(&c,&cfg,&goal,&p,.01f);
            assert(c.output.status!=POSITION_REACHED && sign*c.output.omega_radps<0);
        }
        for(unsigned j=0;j<220 && c.output.status>=0;++j)
            position_control_update(&c,&cfg,&goal,&p,.01f);
        assert(c.output.status==POSITION_YAW_STALLED);
        p=valid_pose(sign*1.78f); p.x_m=-.015f; position_control_reset(&c);
        for(unsigned j=0;j<60;++j) {
            position_control_update(&c,&cfg,&goal,&p,.01f);
            assert(c.output.status!=POSITION_REACHED);
        }
        p=valid_pose(sign*1.78f); position_control_reset(&c);
        unsigned j;
        for(j=0;j<2000;++j) {
            position_control_update(&c,&cfg,&goal,&p,.01f);
            if(c.output.status==POSITION_REACHED) break;
            assert(c.output.status>=0);
            p.yaw_deg+=c.output.omega_radps*.01f*57.2957795f;
        }
        assert(j<2000 && fabsf(p.yaw_deg)<=.5f);
    }
    /* Once aligned, small noise across the inner threshold must not restart
     * friction kicks. Only crossing the outer threshold acquires correction. */
    cfg=(position_config_t)POSITION_CONFIG_DEFAULT;
    position_control_reset(&c); p=valid_pose(.4f);
    for(unsigned j=0;j<200;++j) {
        p.yaw_deg=.48f+.08f*sinf(j*.06f);
        position_control_update(&c,&cfg,&goal,&p,.01f);
        assert(!c.yaw_correcting && fabsf(c.output.omega_radps)<1e-6f);
    }
    p.yaw_deg=1.1f;
    position_control_update(&c,&cfg,&goal,&p,.01f);
    assert(c.yaw_correcting && c.output.omega_radps<0);
    position_control_reset(&c);p=valid_pose(0);
    for(unsigned i=0;i<25;++i) position_control_update(&c,&cfg,&goal,&p,.01f);
    assert(c.output.status==POSITION_REACHED);
    puts("yaw arrival passed: no premature success at 15deg, blocked rotation faults, both-direction alignment, zero-angle settling");
}
