#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_typedef.h"

#define MOTOR1_DIR              (C10)                  //上左
#define MOTOR1_PWM              (PWM2_MODULE2_CHB_C11)

#define MOTOR2_DIR              (D2)                          //上右     
#define MOTOR2_PWM              (PWM2_MODULE3_CHB_D3)            

#define MOTOR3_DIR              (C7)                          //下左
#define MOTOR3_PWM              (PWM2_MODULE0_CHA_C6)

#define MOTOR4_DIR              (C9)                           //下右
#define MOTOR4_PWM              (PWM2_MODULE1_CHA_C8)

/* Positive wheel speed uses these DIR levels. Keep wheel polarity explicit. */
#define MOTOR1_FORWARD_LEVEL         (GPIO_LOW)
#define MOTOR2_FORWARD_LEVEL         (GPIO_LOW)
#define MOTOR3_FORWARD_LEVEL         (GPIO_HIGH)
#define MOTOR4_FORWARD_LEVEL         (GPIO_HIGH)

//下左
#define ENCODER_1                   (QTIMER1_ENCODER1)
#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)
//下右
#define ENCODER_2                   (QTIMER1_ENCODER2)
#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)
//上右
#define ENCODER_3                   (QTIMER2_ENCODER1)
#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C4)
//上左
#define ENCODER_4                   (QTIMER2_ENCODER2)
#define ENCODER_4_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_4_B                 (QTIMER2_ENCODER2_CH2_C25)

/* Convert physical counts to positive forward wheel feedback. */
#define ENCODER_1_FORWARD_SIGN       (1)
#define ENCODER_2_FORWARD_SIGN       (1)   //DR：按实测正 PWM 方向修正反馈符号
#define ENCODER_3_FORWARD_SIGN       (-1)
#define ENCODER_4_FORWARD_SIGN       (1)

//参数宏定义
#define ENCODER_GEAR_RATIO      2.333f     //电机轴转数 / 车轮转数，当前近似减速比
/* Backend counts A rising edges (1x), not all four quadrature edges. */
#define ENCODER_LINES_UL        1024.0f
#define ENCODER_LINES_UR        1024.0f
#define ENCODER_LINES_DL        1024.0f
#define ENCODER_LINES_DR        512.0f
#define ENCODER_RESOLUTION_UL   (ENCODER_LINES_UL * ENCODER_GEAR_RATIO)
#define ENCODER_RESOLUTION_UR   (ENCODER_LINES_UR * ENCODER_GEAR_RATIO)
#define ENCODER_RESOLUTION_DL   (ENCODER_LINES_DL * ENCODER_GEAR_RATIO)
#define ENCODER_RESOLUTION_DR   (ENCODER_LINES_DR * ENCODER_GEAR_RATIO)
/* PID targets use the UL reference resolution; feedback is normalized to it. */
#define ENCODER_RESOLUTION      ENCODER_RESOLUTION_UL
#define WHEEL_DIAMETER          0.063     //轮子直径,单位：米（6.3 cm）
#define LATERAL_CORRECTION_FACTOR 0.901589f  //实际横移距离 / 计划横移距离
#define LATERAL_TO_LONGITUDINAL_COUPLING_FACTOR 0.0f  // dx drift / dy travel 单位横移速度产生的额外前后速度
#define D_X                     0.176     //底盘Y轴上两轮中心的间距
#define D_Y                     0.20     //底盘X轴上两轮中心的间距
#define PID_RATE                100       //PID调节PWM值的频率

#define LIMIT_PWM_MIN              -2000
#define LIMIT_PWM_MAX               2000

/* Closed-loop motor parameters in logical order: UL, UR, DL, DR. */
#define MOTOR_DEADZONE_TARGET_MIN_COUNTS  2

#define MOTOR_UL_DEADZONE_FWD             420
#define MOTOR_UL_DEADZONE_REV             390
#define MOTOR_UR_DEADZONE_FWD             495
#define MOTOR_UR_DEADZONE_REV             429
#define MOTOR_DL_DEADZONE_FWD             435
#define MOTOR_DL_DEADZONE_REV             390
#define MOTOR_DR_DEADZONE_FWD             550
#define MOTOR_DR_DEADZONE_REV             637

/* Right-strafe launch compensation, limited to the first 5 cm.
 * Derive distance counts from the configured wheel diameter/resolution. */
#define MOTOR_RIGHT_START_MIN_TARGET_COUNTS       5
#define MOTOR_RIGHT_START_MAX_FORWARD_COUNTS      4
#define MOTOR_RIGHT_START_REVERSE_COUNTS          1
#define MOTOR_RIGHT_START_DISTANCE_COUNTS         ((int)(ENCODER_RESOLUTION * 0.05f / (WHEEL_DIAMETER * 3.1415926f) + 0.5f))
#define MOTOR_RIGHT_START_MAX_TICKS               50U
#define MOTOR_RIGHT_START_REARM_TICKS              5U

#define MOTOR_WHEEL_COUNT                  4
#define MOTOR_WHEEL_UL                     0
#define MOTOR_WHEEL_UR                     1
#define MOTOR_WHEEL_DL                     2
#define MOTOR_WHEEL_DR                     3

typedef struct
{
    /* Targets are reference counts; raw/filtered/total are physical counts. */
    int target_counts[MOTOR_WHEEL_COUNT];
    int raw_counts[MOTOR_WHEEL_COUNT];
    int filtered_counts[MOTOR_WHEEL_COUNT];
    int pid_pwm[MOTOR_WHEEL_COUNT];
    int final_pwm[MOTOR_WHEEL_COUNT];
    int32 cumulative_target_counts[MOTOR_WHEEL_COUNT];
    int32 cumulative_raw_counts[MOTOR_WHEEL_COUNT];
    uint32 control_ticks;
    float wheel_speed_cmps[MOTOR_WHEEL_COUNT];
} motor_speed_debug_snapshot_t;

#define LIMIT_ENCODER_MIN          -500
#define LIMIT_ENCODER_MAX           500
#define ENCODER_FILTER_ALPHA       0.35f


extern int all;
extern int16 up_L_all;
extern int16 down_L_all;
extern int16 up_R_all;
extern int16 down_R_all;

extern int32 encoder_all;
extern int32 encoders_average;
/* Logical wheel counts after direction correction: UL, UR, DL, DR. */
extern int16 encoder_data_quaddec1;
extern int16 encoder_data_quaddec2;
extern int16 encoder_data_quaddec3;
extern int16 encoder_data_quaddec4;
extern double pulse_per_meter;
extern const float motor_encoder_counts_per_revolution[MOTOR_WHEEL_COUNT];
extern float rx_plus_ry_cali;

extern float speed_three_array[3];
extern int speed_encoder[4];
extern int car_stop_array[4];
extern volatile int motor_deadzone_target_min_counts;
extern volatile int motor_deadzone_fwd[MOTOR_WHEEL_COUNT];
extern volatile int motor_deadzone_rev[MOTOR_WHEEL_COUNT];

void motor_init(void);
void encoder_init(void);
void encoder_get(void);
int Limit_int(int left_limit, int target_num, int right_limit);
void motor_pwm(int up_left_speed,int up_right_speed,int down_left_speed,int down_right_speed);
void motor_control(int* input_speed_encoder);
void motor_limit_right_start_forward_offset(const int *input_speed_encoder,
                                            int *limited_speed_encoder);
void motor_right_start_compensation_reset(void);
void motor_speed_debug_reset(void);
void motor_speed_debug_get_snapshot(motor_speed_debug_snapshot_t *snapshot);
//void encoder_read_filtered(int *enc1, int *enc2, int *enc3, int *enc4);
float Lowpass(float X_last, float X_new);
/* Input is one wheel's physical increment during a 10 ms control period. */
float motor_encoder_counts_to_cmps(uint8 wheel, float counts);
/* Normalize physical counts to the resolution used by the wheel PID targets. */
float motor_reference_counts(uint8 wheel, float physical_counts);
void Kinematics_Init(void);
void Kinematics_Inverse(float* input, int* output);

#endif
