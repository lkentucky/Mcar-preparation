#include "scurve_profile.h"
#include "odometry_calibration.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void check(float distance,float v0,float v1,float vmax,float amax,float jmax)
{
    scurve_profile_t p;
    assert(scurve_profile_build(&p,distance,v0,v1,vmax,amax,jmax));
    scurve_sample_t before=scurve_profile_sample(&p,0);
    assert(fabsf(before.velocity_cmps-v0)<1e-4f);
    double area=0;
    float dt=p.total_s/10000;
    for(unsigned i=1;i<=10000;++i) {
        scurve_sample_t s=scurve_profile_sample(&p,i*dt);
        assert(s.velocity_cmps>=-1e-3f && s.velocity_cmps<=vmax+1e-3f);
        assert(fabsf(s.acceleration_cmps2)<=amax+1e-3f);
        assert(s.position_cm>=before.position_cm-1e-3f);
        /* On long float32 timelines, the represented time increment differs
         * slightly from dt. Bound jerk against the actual sampled increment. */
        float sampled_dt=i*dt-(i-1)*dt;
        assert(fabsf(s.acceleration_cmps2-before.acceleration_cmps2)<=jmax*sampled_dt+.02f);
        area+=.5*(before.velocity_cmps+s.velocity_cmps)*dt;
        before=s;
    }
    assert(fabs(area-distance)<.003+distance*1e-5);
    scurve_sample_t end=scurve_profile_sample(&p,p.total_s+1);
    assert(end.complete && end.phase==8 && end.position_cm==distance && end.velocity_cmps==v1);
    for(unsigned i=0;i<7;++i) assert(p.duration[i]>=0);
}
int main(void)
{
    check(.01f,0,0,20,40,120);
    check(1,0,0,20,40,120);
    check(100,0,0,20,40,120);
    check(10000,0,0,100,300,3000);
    check(100,5,0,20,40,120);
    check(100,0,5,20,40,120);
    check(100,5,3,20,40,120);
    check(1,10,10,20,40,120);
    scurve_profile_t p;
    assert(!scurve_profile_build(&p,1,20,0,20,40,120)); /* Cannot fit full braking ramp. */
    assert(!scurve_profile_build(&p,10,0,0,20,40,NAN));
    assert(!scurve_profile_build(&p,10,0,0,20,40,0));
    assert(scurve_profile_build(&p,0,0,0,20,40,120));
    assert(scurve_profile_sample(&p,0).complete);
    assert(scurve_profile_sample(NULL,0).phase==0);
    float scale=-1;
    assert(odometry_calibration_scale(.52f,100,50,&scale) && fabsf(scale-.26f)<1e-6f);
    assert(odometry_calibration_scale(.61f,-100,60,&scale) && fabsf(scale-.366f)<1e-6f);
    assert(!odometry_calibration_scale(1,0,100,&scale));
    assert(!odometry_calibration_scale(1,100,NAN,&scale));
    assert(!odometry_calibration_scale(1,100,5,&scale));
    assert(!odometry_calibration_scale(1,10,100,&scale));
    puts("S-curve passed: short/long/asymmetric profiles, distance integration, v/a/jerk bounds; calibration ratios");
    return 0;
}
