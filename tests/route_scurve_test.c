#include "route_follow.h"
#include "app_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

volatile bool motor_run_enabled,motor_position_enabled,motor_pwm_test_enabled;
volatile position_goal_t motor_position_goal;
static position_control_t controller;
void app_control_get_position_snapshot(position_output_t *out) { *out=controller.output; }
int main(void)
{
    position_config_t config=POSITION_CONFIG_DEFAULT;
    navigation_snapshot_t pose={.valid=true,.bias_ready=true,.status=NAV_RUNNING};
    route_node_count=3;
    route_nodes[0]=(route_node_t){0,0,0};
    route_nodes[1]=(route_node_t){20,40,0};
    route_nodes[2]=(route_node_t){50,80,0};
    route_follow_init();route_follow_start();
    unsigned maximum_index=0, i;
    for(i=0;i<4000;++i) {
        if(motor_run_enabled && motor_position_enabled) {
            position_goal_t goal=motor_position_goal;
            position_control_update(&controller,&config,&goal,&pose,.01f);
            if(controller.output.status==POSITION_REACHED || controller.output.status<0) motor_run_enabled=false;
        }
        route_follow_tick_10ms();
        if((unsigned)route_current_idx>maximum_index) maximum_index=(unsigned)route_current_idx;
        float vx=motor_run_enabled?controller.output.vx_cmps*.01f:0;
        float vy=motor_run_enabled?controller.output.vy_cmps*.01f:0;
        pose.vx_mps+=.2f*(vx-pose.vx_mps);pose.vy_mps+=.2f*(vy-pose.vy_mps);
        pose.x_m+=pose.vx_mps*.01f;pose.y_m+=pose.vy_mps*.01f;
        if(route_state==ROUTE_DONE) break;
    }
    assert(i<4000 && maximum_index==2 && !motor_run_enabled && !route_run_flag);
    assert(hypotf(pose.x_m*100-50,pose.y_m*100-80)<=config.xy_tolerance_cm+.05f);
    route_follow_start();pose.valid=false;
    for(i=0;i<10;++i) {
        if(motor_run_enabled) {
            position_goal_t goal=motor_position_goal;
            position_control_update(&controller,&config,&goal,&pose,.01f);
            if(controller.output.status<0) motor_run_enabled=false;
        }
        route_follow_tick_10ms();
    }
    assert(route_state==ROUTE_FAULT && !motor_run_enabled && !route_run_flag);
    puts("route S-curve passed: real position planner, sequential convergence, actual arrival, pose fault stops route");
    return 0;
}
