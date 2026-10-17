#define main legacy_motor_main
#include "motor_mapping_test.c"
#undef main
void imu_track_gyro_bias(float stationary_seconds,float dt_s) { (void)stationary_seconds; (void)dt_s; }
int main(void)
{
    navigation_snapshot_t pose;
    position_output_t out;
    app_control_init();
    memset(counts,0,sizeof(counts));
    memset(&fake_imu,0,sizeof(fake_imu));
    fake_imu.accel_g[2]=1; fake_imu.dt_s=.005f; fake_imu.generation=1;
    imu_attitude_status=IMU_ATTITUDE_RUNNING;
    for (unsigned i=0;i<60;++i) position_tick();
    app_navigation_get_snapshot(&pose); assert(pose.valid);
    motor_pwm_test_enabled=true; motor_position_enabled=false; position_tick();
    app_control_remote_goal(50,0,0);
    position_tick(); assert_position_stopped(); /* mode transition consumes one tick */
    position_tick();
    app_control_get_position_snapshot(&out);
    assert(motor_position_enabled && !motor_pwm_test_enabled && motor_run_enabled);
    assert(out.status==POSITION_MOVING && motor_cmd_vx_cmps>0);
    app_control_remote_goal(0,50,0); position_tick();
    assert(motor_run_enabled && motor_position_goal.y_cm==50);
    app_control_remote_stop(); assert_position_stopped();
    for(unsigned i=0;i<4;++i) position_tick();
    assert_position_stopped();
    app_control_remote_goal(25,25,0); app_control_remote_stop(); position_tick(); assert_position_stopped();
    imu_attitude_status=IMU_ATTITUDE_FILTER_ERROR;
    app_control_remote_goal(50,0,0); position_tick(); assert_position_stopped();
    puts("remote motor integration passed: automatic mode switch, goal replace, immediate STOP, cancel pending, invalid pose");
    return 0;
}

