#include "app_navigation.h"
#include "app_control.h"
#include "imu.h"
#include "Motor.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

volatile bool motor_run_enabled;
bool route_run_flag;
volatile int32 imu_attitude_status=IMU_ATTITUDE_RUNNING;
int16 encoder_data_quaddec1,encoder_data_quaddec2,encoder_data_quaddec3,encoder_data_quaddec4;
const float motor_encoder_counts_per_revolution[4]={2388.992f,2388.992f,2388.992f,1194.496f};
static imu_navigation_sample_t sample={.generation=1,.dt_s=.005f,.accel_g={0,0,1}};
static unsigned locked;
uint32 interrupt_global_disable(void) { assert(!locked);locked=1;return 0; }
void interrupt_global_enable(uint32 p) { assert(locked && p==0);locked=0; }
bool imu_get_navigation_sample(imu_navigation_sample_t *out) { *out=sample;return true; }
static void tick(void)
{
    for(unsigned i=0;i<2;++i) { ++sample.sequence;app_navigation_imu_tick_5ms(); }
    app_navigation_encoder_tick_10ms();
}
static void stationary(void)
{
    encoder_data_quaddec1=encoder_data_quaddec2=encoder_data_quaddec3=encoder_data_quaddec4=0;
    for(unsigned i=0;i<100;++i) tick();
}
int main(void)
{
    navigation_snapshot_t pose;
    app_navigation_init();
    assert(app_navigation_calibrate(0,100)==-3);
    stationary();
    for(unsigned i=0;i<100;++i) {
        encoder_data_quaddec1=encoder_data_quaddec2=encoder_data_quaddec3=20;
        encoder_data_quaddec4=10; tick();
    }
    stationary();app_navigation_get_snapshot(&pose);
    assert(pose.valid && pose.bias_ready && pose.x_m>.1f && fabsf(pose.y_m)<.01f);
    float old=navigation_scale_x, measured=pose.x_m*100*.8f;
    motor_run_enabled=true;assert(app_navigation_calibrate(0,measured)==-2);
    assert(navigation_scale_x==old); motor_run_enabled=false;
    route_run_flag=true;assert(app_navigation_calibrate(0,measured)==-2);route_run_flag=false;
    assert(app_navigation_calibrate(2,measured)==-1);
    assert(app_navigation_calibrate(0,NAN)==-1);
    assert(app_navigation_calibrate(0,measured)==1);
    assert(fabsf(navigation_scale_x-old*.8f)<1e-5f);
    assert(app_navigation_calibrate(0,measured)==-3); /* A duplicate before Zero cannot apply the ratio twice. */
    assert(fabsf(navigation_scale_x-old*.8f)<1e-5f);
    tick();app_navigation_get_snapshot(&pose);assert(pose.x_m==0 && pose.y_m==0);
    stationary();
    for(unsigned i=0;i<100;++i) {
        encoder_data_quaddec1=-20;encoder_data_quaddec2=encoder_data_quaddec3=20;
        encoder_data_quaddec4=-10;tick();
    }
    stationary();app_navigation_get_snapshot(&pose);
    assert(pose.y_m>.1f && fabsf(pose.x_m)<.01f);
    old=navigation_scale_y;
    assert(app_navigation_calibrate(1,pose.y_m*100*1.1f)==1);
    assert(fabsf(navigation_scale_y-old*1.1f)<1e-5f);
    tick();stationary();
    sample.yaw_deg=10;stationary();
    assert(app_navigation_calibrate(0,100)==-4);
    assert(!motor_run_enabled && !locked);
    puts("calibration adapter passed: real encoder/fusion, X/Y update, Zero, Run/pose/heading rejection");
    return 0;
}
