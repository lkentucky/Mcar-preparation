/** 姿态算法独立于 SPI、屏幕和 WiFi；保留原算法计算顺序及数值门限。
 * Madgwick 来源说明：Sebastian O.H. Madgwick IMU 算法报告
 * https://x-io.co.uk/downloads/madgwick_internal_report.pdf
 * 原项目实现声明 GPL-3.0-or-later；本次合并保留该声明。
 */
#include "attitude.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

/* 避免快速浮点优化将 NaN/Inf 比较优化掉；memcpy 避免别名访问。 */
static int finite_float(float value)
{
    uint32_t bits;
    typedef char float_size_check[(sizeof(float)==sizeof(uint32_t)) ? 1 : -1];
    (void)sizeof(float_size_check);
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}
static float attitude_clamp(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}
void calibration_reset(ImuCalibration *c)
{
    memset(c, 0, sizeof(*c));
}
/* 400 个连续合格样本，200Hz 下约 2 秒；不能识别所有匀速运动。 */
int calibration_push(ImuCalibration *c, const float g[3], const float a[3])
{
    unsigned i;
    float n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(n) || n < 0.9f || n > 1.1f) {
        calibration_reset(c); return 0;
    }
    for (i=0; i<3; ++i) {
        if (!finite_float(g[i]) || fabsf(g[i]) > CAL_GYRO_LIMIT_DPS) {
            calibration_reset(c); return 0;
        }
        if (!c->count) {
            c->min_g[i]=c->max_g[i]=g[i]; c->min_a[i]=c->max_a[i]=a[i];
        }
        if (g[i]<c->min_g[i]) c->min_g[i]=g[i];
        if (g[i]>c->max_g[i]) c->max_g[i]=g[i];
        if (a[i]<c->min_a[i]) c->min_a[i]=a[i];
        if (a[i]>c->max_a[i]) c->max_a[i]=a[i];
        /* 峰峰值超限：丢弃整个静止窗口。 */
        if (c->max_g[i]-c->min_g[i]>CAL_GYRO_SPAN_DPS ||
            c->max_a[i]-c->min_a[i]>CAL_ACCEL_SPAN_G) {
            calibration_reset(c); return 0;
        }
    }
    for (i=0; i<3; ++i) { c->sum_g[i]+=g[i]; c->sum_a[i]+=a[i]; }
    if (++c->count < CALIBRATION_SAMPLES) return 0;
    for (i=0; i<3; ++i) {
        c->bias_dps[i]=c->sum_g[i]/c->count;
        c->initial_accel[i]=c->sum_a[i]/c->count;
    }
    return 1;
}
int attitude_init(float q[4], float *accel_norm, const float a[3])
{
    float norm, roll, pitch, cr, sr, cp, sp;
    if (!finite_float(a[0]) || !finite_float(a[1]) || !finite_float(a[2])) return 0;
    norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(norm) || norm < ACC_NORM_MIN || norm > ACC_NORM_MAX) return 0;
    roll=atan2f(a[1],a[2]);
    pitch=atan2f(-a[0],sqrtf(a[1]*a[1]+a[2]*a[2]));
    cr=cosf(roll*0.5f); sr=sinf(roll*0.5f);
    cp=cosf(pitch*0.5f); sp=sinf(pitch*0.5f);
    q[0]=cr*cp; q[1]=sr*cp; q[2]=cr*sp; q[3]=-sr*sp;
    *accel_norm=norm;
    return 1;
}
/* R=Rz(yaw)*Ry(pitch)*Rx(roll)，pitch 在 +/-90 度处存在万向锁。 */
void attitude_euler(const float q[4], float e[3])
{
    float w=q[0],x=q[1],y=q[2],z=q[3];
    e[0]=atan2f(2.0f*(w*x+y*z),1.0f-2.0f*(x*x+y*y))*RAD_TO_DEG;
    e[1]=asinf(attitude_clamp(2.0f*(w*y-z*x),-1.0f,1.0f))*RAD_TO_DEG;
    e[2]=atan2f(2.0f*(w*z+x*y),1.0f-2.0f*(y*y+z*z))*RAD_TO_DEG;
}
void attitude_cube_euler(const float q[4], float e[3])
{
    float w=q[0],x=q[1],y=q[2],z=q[3];
    float r00=1.0f-2.0f*(y*y+z*z),r02=2.0f*(x*z+w*y);
    float r10=2.0f*(x*y+w*z),r11=1.0f-2.0f*(x*x+z*z);
    float r12=2.0f*(y*z-w*x),r20=2.0f*(x*z-w*y);
    float r22=1.0f-2.0f*(x*x+y*y);
    float cx=sqrtf(r10*r10+r11*r11);
    e[0]=atan2f(-r12,cx)*RAD_TO_DEG;
    if (cx>1e-5f) {
        e[1]=atan2f(r02,r22)*RAD_TO_DEG;
        e[2]=atan2f(r10,r11)*RAD_TO_DEG;
    } else {
        e[1]=atan2f(-r20,r00)*RAD_TO_DEG;
        e[2]=0;
    }
}
int mahony6_init(Mahony6 *s, const float a[3])
{
    Mahony6 next={0};
    if (!attitude_init(next.q,&next.accel_norm,a)) return 0;
    *s=next; return 1;
}
/* Mahony：实测重力叉积预测重力，PI 修正角速度；在副本上计算后提交。 */
int mahony6_update(Mahony6 *s, const float g[3], const float a[3], float dt)
{
    Mahony6 next=*s;
    float w=s->q[0],x=s->q[1],y=s->q[2],z=s->q[3];
    float omega[3],error[3]={0,0,0},n;
    unsigned i;
    if (!finite_float(dt) || dt<=0.0f || dt>MAX_SAMPLE_DT) return 0;
    for (i=0;i<3;++i) if (!finite_float(g[i]) || !finite_float(a[i])) return 0;
    n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(n)) return 0;
    next.accel_norm=n;
    next.accel_used=(n>=ACC_NORM_MIN && n<=ACC_NORM_MAX);
    if (next.accel_used) {
        float ax=a[0]/n,ay=a[1]/n,az=a[2]/n;
        float vx=2.0f*(x*z-w*y),vy=2.0f*(w*x+y*z),vz=w*w-x*x-y*y+z*z;
        error[0]=ay*vz-az*vy;
        error[1]=az*vx-ax*vz;
        error[2]=ax*vy-ay*vx;
    }
    for (i=0;i<3;++i) {
        if (AHRS_KI>0.0f) {
            /* 加速度不可信时冻结积分，仍保留已有积分修正。 */
            if (next.accel_used)
                next.integral[i]=attitude_clamp(next.integral[i]+AHRS_KI*error[i]*dt,
                                                -AHRS_INTEGRAL_LIMIT,AHRS_INTEGRAL_LIMIT);
        } else next.integral[i]=0.0f;
        omega[i]=g[i]+AHRS_KP*error[i]+next.integral[i];
    }
    /* 常值角速度下的轴角精确增量：q_new=q*dq。 */
    {
        float mag=sqrtf(omega[0]*omega[0]+omega[1]*omega[1]+omega[2]*omega[2]);
        float theta=0.5f*mag*dt;
        float dq0,scale,dq1,dq2,dq3;
        if (!finite_float(mag) || !finite_float(theta)) return 0;
        dq0=cosf(theta);
        scale=(mag>1e-9f)?sinf(theta)/mag:0.5f*dt;
        dq1=omega[0]*scale; dq2=omega[1]*scale; dq3=omega[2]*scale;
        next.q[0]=w*dq0-x*dq1-y*dq2-z*dq3;
        next.q[1]=w*dq1+x*dq0+y*dq3-z*dq2;
        next.q[2]=w*dq2-x*dq3+y*dq0+z*dq1;
        next.q[3]=w*dq3+x*dq2-y*dq1+z*dq0;
    }
    n=sqrtf(next.q[0]*next.q[0]+next.q[1]*next.q[1]+next.q[2]*next.q[2]+next.q[3]*next.q[3]);
    if (!finite_float(n) || n<1e-6f) return 0;
    for (i=0;i<4;++i) next.q[i]/=n;
    *s=next; return 1;
}
void mahony6_euler(const Mahony6 *s,float e[3]) { attitude_euler(s->q,e); }
void mahony6_cube_euler(const Mahony6 *s,float e[3]) { attitude_cube_euler(s->q,e); }

int madgwick6_init(Madgwick6 *s,const float a[3])
{
    Madgwick6 next={0};
    if (!attitude_init(next.q,&next.accel_norm,a)) return 0;
    *s=next; return 1;
}
/* Madgwick：qdot=陀螺项-beta*归一化梯度，再按实际 dt 积分。 */
int madgwick6_update(Madgwick6 *s,const float g[3],const float a[3],float dt)
{
    Madgwick6 next=*s;
    float w=s->q[0],x=s->q[1],y=s->q[2],z=s->q[3];
    float qdot[4],n;
    unsigned i;
    if (!finite_float(dt)||dt<=0.0f||dt>MAX_SAMPLE_DT||
        !finite_float(MADGWICK_BETA)||MADGWICK_BETA<0.0f) return 0;
    for (i=0;i<3;++i) if (!finite_float(g[i])||!finite_float(a[i])) return 0;
    for (i=0;i<4;++i) if (!finite_float(s->q[i])) return 0;
    n=w*w+x*x+y*y+z*z;
    if (!finite_float(n)||n<1e-12f) return 0;
    qdot[0]=0.5f*(-x*g[0]-y*g[1]-z*g[2]);
    qdot[1]=0.5f*(w*g[0]+y*g[2]-z*g[1]);
    qdot[2]=0.5f*(w*g[1]-x*g[2]+z*g[0]);
    qdot[3]=0.5f*(w*g[2]+x*g[1]-y*g[0]);
    n=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
    if (!finite_float(n)) return 0;
    next.accel_norm=n;
    next.accel_used=(n>=ACC_NORM_MIN&&n<=ACC_NORM_MAX);
    if (next.accel_used&&MADGWICK_BETA>0.0f) {
        float ax=a[0]/n,ay=a[1]/n,az=a[2]/n;
        float f0=2.0f*(x*z-w*y)-ax;
        float f1=2.0f*(w*x+y*z)-ay;
        float f2=1.0f-2.0f*(x*x+y*y)-az;
        float step[4]={-2.0f*y*f0+2.0f*x*f1,
                      2.0f*z*f0+2.0f*w*f1-4.0f*x*f2,
                      -2.0f*w*f0+2.0f*z*f1-4.0f*y*f2,
                      2.0f*x*f0+2.0f*y*f1};
        float step_norm=sqrtf(step[0]*step[0]+step[1]*step[1]+step[2]*step[2]+step[3]*step[3]);
        if (!finite_float(step_norm)) return 0;
        /* 重力一致时梯度为零，避免除零。 */
        if (step_norm>1e-6f)
            for (i=0;i<4;++i) qdot[i]-=MADGWICK_BETA*step[i]/step_norm;
    }
    for (i=0;i<4;++i) next.q[i]=s->q[i]+qdot[i]*dt;
    n=sqrtf(next.q[0]*next.q[0]+next.q[1]*next.q[1]+next.q[2]*next.q[2]+next.q[3]*next.q[3]);
    if (!finite_float(n)||n<1e-6f) return 0;
    for (i=0;i<4;++i) next.q[i]/=n;
    *s=next; return 1;
}
void madgwick6_euler(const Madgwick6 *s,float e[3]) { attitude_euler(s->q,e); }
void madgwick6_cube_euler(const Madgwick6 *s,float e[3]) { attitude_cube_euler(s->q,e); }
