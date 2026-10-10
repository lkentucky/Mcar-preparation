#include "scurve_profile.h"
#include <math.h>
#include <string.h>

typedef struct { float tj, constant, total; } ramp_t;
static ramp_t ramp(float delta_v, float amax, float jmax)
{
    ramp_t r = {0};
    if (delta_v <= 0) return r;
    r.tj = fminf(amax / jmax, sqrtf(delta_v / jmax));
    r.constant = fmaxf(0, delta_v / (jmax * r.tj) - r.tj);
    r.total = 2*r.tj + r.constant;
    return r;
}
static float ramp_distance(float peak, float v0, float v1, float amax, float jmax)
{
    ramp_t up = ramp(peak-v0, amax, jmax), down = ramp(peak-v1, amax, jmax);
    return .5f*(v0+peak)*up.total + .5f*(v1+peak)*down.total;
}
bool scurve_profile_build(scurve_profile_t *p, float distance, float v0, float v1,
                          float vmax, float amax, float jmax)
{
    float low, high, peak, cruise=0, s=0, v=v0, a=0;
    ramp_t up, down;
    if (!p) return false;
    memset(p,0,sizeof(*p));
    if (!isfinite(distance) || !isfinite(v0) || !isfinite(v1) || !isfinite(vmax) ||
        !isfinite(amax) || !isfinite(jmax) || distance<0 || v0<0 || v1<0 ||
        vmax<=0 || amax<=0 || jmax<=0 || v0>vmax || v1>vmax) return false;
    low = fmaxf(v0,v1); high = vmax;
    if (ramp_distance(low,v0,v1,amax,jmax)>distance+1e-4f) return false;
    if (distance <= 1e-6f) {
        if (v0>1e-6f || v1>1e-6f) return false;
        p->valid=true; return true;
    }
    if (ramp_distance(vmax,v0,v1,amax,jmax)<=distance) {
        peak=vmax;
        cruise=(distance-ramp_distance(peak,v0,v1,amax,jmax))/peak;
    } else {
        /* Monotonic distance/peak relation; fixed iterations, no unbounded loop. */
        for (unsigned i=0;i<32;++i) {
            float middle=.5f*(low+high);
            if (ramp_distance(middle,v0,v1,amax,jmax)>distance) high=middle;
            else low=middle;
        }
        peak=.5f*(low+high);
    }
    up=ramp(peak-v0,amax,jmax); down=ramp(peak-v1,amax,jmax);
    p->duration[0]=up.tj; p->duration[1]=up.constant; p->duration[2]=up.tj;
    p->duration[3]=cruise;
    p->duration[4]=down.tj; p->duration[5]=down.constant; p->duration[6]=down.tj;
    p->jerk[0]=jmax; p->jerk[2]=-jmax; p->jerk[4]=-jmax; p->jerk[6]=jmax;
    for (unsigned i=0;i<7;++i) {
        float t=p->duration[i], j=p->jerk[i];
        p->start_s[i]=s; p->start_v[i]=v; p->start_a[i]=a;
        s+=v*t+.5f*a*t*t+j*t*t*t/6;
        v+=a*t+.5f*j*t*t; a+=j*t; p->total_s+=t;
    }
    p->distance_cm=distance; p->end_speed_cmps=v1;
    p->valid=isfinite(p->total_s) && p->total_s>0;
    return p->valid;
}
scurve_sample_t scurve_profile_sample(const scurve_profile_t *p, float time)
{
    scurve_sample_t out={0};
    if (!p || !p->valid || !isfinite(time)) return out;
    if (time>=p->total_s) {
        out.position_cm=p->distance_cm; out.velocity_cmps=p->end_speed_cmps;
        out.phase=8; out.complete=true; return out;
    }
    time=fmaxf(time,0);
    for (unsigned i=0;i<7;++i) {
        if (time<p->duration[i]) {
            float j=p->jerk[i];
            /* Accumulate the position polynomial in double to avoid small
             * backwards roundoff near a long segment's zero-speed endpoint. */
            double t=time;
            out.position_cm=(float)(p->start_s[i]+p->start_v[i]*t+.5*p->start_a[i]*t*t+j*t*t*t/6);
            out.position_cm=fminf(p->distance_cm,fmaxf(0,out.position_cm));
            out.velocity_cmps=p->start_v[i]+p->start_a[i]*time+.5f*j*time*time;
            out.acceleration_cmps2=p->start_a[i]+j*time;
            out.phase=i+1; return out;
        }
        time-=p->duration[i];
    }
    out.position_cm=p->distance_cm; out.velocity_cmps=p->end_speed_cmps;
    out.phase=8; out.complete=true; return out;
}
