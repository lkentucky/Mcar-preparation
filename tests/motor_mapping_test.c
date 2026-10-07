/* Test the actual driver with recorded hardware I/O, without energizing motors.
 * Build with motor_stubs before the real zf_driver include directory.
 */
#include "zf_driver_gpio.h"
#include "zf_driver_pwm.h"
#include "zf_driver_encoder.h"
#include "Motor.h"
#include "PID_config.h"
#include "app_control.h"
#include "app_navigation.h"
#include "imu_attitude.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <math.h>

static uint32 duties[256];
static uint8 levels[256];
static int16 counts[16];
static unsigned clear_calls[16];
static unsigned motor_init_calls;
static unsigned pwm_init_calls;
static unsigned encoder_init_calls;
volatile int32 imu_attitude_status;
static imu_navigation_sample_t fake_imu;

bool imu_attitude_get_navigation_sample(imu_navigation_sample_t *out)
{
    if (imu_attitude_status != IMU_ATTITUDE_RUNNING) return false;
    *out = fake_imu;
    return true;
}

static void imu_tick(void)
{
    ++fake_imu.sequence;
    app_navigation_imu_tick_5ms();
}

static void check_navigation_integration(void)
{
    navigation_snapshot_t out;
    app_navigation_init();
    assert(navigation_mount_deg == 180.0f && !navigation_yaw_reversed);
    memset(&fake_imu, 0, sizeof(fake_imu));
    fake_imu.accel_g[2] = 1.0f;
    fake_imu.dt_s = 0.005f;
    fake_imu.generation = 1;
    imu_attitude_status = IMU_ATTITUDE_RUNNING;
    motor_pwm_test_enabled = true;
    motor_run_enabled = false;
    memset(counts, 0, sizeof(counts));
    /* Fusion is updated even while PWM mode is selected and Run is off. */
    for (unsigned i = 0; i < 55; ++i) {
        imu_tick(); imu_tick(); app_control_motor_tick_10ms();
    }
    app_navigation_get_snapshot(&out);
    assert(out.valid && out.bias_ready);
    for (unsigned i = 0; i < 100; ++i) {
        counts[ENCODER_1] = 2; counts[ENCODER_2] = 1;
        counts[ENCODER_3] = -2; counts[ENCODER_4] = 2;
        imu_tick(); imu_tick(); app_control_motor_tick_10ms();
    }
    app_navigation_get_snapshot(&out);
    float expected = 200.0f * 3.1415926f * WHEEL_DIAMETER / ENCODER_RESOLUTION_UL;
    assert(out.valid && fabsf(out.x_m - expected) < 1e-5f && fabsf(out.y_m) < 1e-5f);
    for (unsigned i = 0; i < 256; ++i) assert(duties[i] == 0);
    memset(counts, 0, sizeof(counts));
    /* Reading one published IMU frame repeatedly must not extend freshness. */
    for (unsigned i = 0; i < 7; ++i) {
        app_navigation_imu_tick_5ms(); app_control_motor_tick_10ms();
    }
    app_navigation_get_snapshot(&out);
    assert(!out.valid && out.status == NAV_IMU_FAULT);
    app_navigation_request_reset(); app_control_motor_tick_10ms();
    for (unsigned i = 0; i < 55; ++i) { imu_tick(); imu_tick(); app_control_motor_tick_10ms(); }
    app_navigation_get_snapshot(&out);
    assert(out.valid && out.x_m == 0.0f);
    ++fake_imu.generation; imu_tick();
    app_navigation_get_snapshot(&out);
    assert(!out.valid && out.status == NAV_IMU_FAULT);
    puts("navigation integration passed: actual encoder adapter, stopped/PWM sampling, stale frame, Zero, AHRS recalibration");
}

/* Independent fixture from the updated wiring, in UL/UR/DL/DR order. */
static const gpio_pin_enum expected_dir[] = {D13, D12, D0, D1};
static const pwm_channel_enum expected_pwm[] = {
    PWM1_MODULE1_CHB_D15, PWM1_MODULE1_CHA_D14,
    PWM2_MODULE3_CHA_D2, PWM2_MODULE3_CHB_D3
};
/* Forward DIR levels follow the user's current hardware setting. */
static const uint8 expected_forward[] = {GPIO_LOW, GPIO_LOW, GPIO_HIGH, GPIO_HIGH};

void gpio_set_level(gpio_pin_enum pin, uint8 level)
{
    assert((unsigned)pin < 256);
    levels[pin] = level;
}

void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8 level, uint32 config)
{
    assert(motor_init_calls < 4);
    assert(pin == expected_dir[motor_init_calls]);
    assert(dir == GPO && config == GPO_PUSH_PULL);
    assert(level == expected_forward[motor_init_calls]);
    motor_init_calls++;
    gpio_set_level(pin, level);
}

void pwm_set_duty(pwm_channel_enum pin, const uint32 duty)
{
    assert((unsigned)pin < 256);
    assert(duty <= (uint32)(LIMIT_PWM_MAX > -LIMIT_PWM_MIN ? LIMIT_PWM_MAX : -LIMIT_PWM_MIN));
    duties[pin] = duty;
}

void pwm_init(pwm_channel_enum pin, const uint32 freq, const uint32 duty)
{
    assert(pwm_init_calls < 4);
    assert(pin == expected_pwm[pwm_init_calls]);
    assert(freq == 17000 && duty == 0);
    pwm_init_calls++;
    pwm_set_duty(pin, duty);
}

void encoder_quad_init(encoder_index_enum index,
                       encoder_channel1_enum a, encoder_channel2_enum b)
{
    static const encoder_index_enum expected_index[] = {
        QTIMER1_ENCODER1, QTIMER1_ENCODER2, QTIMER2_ENCODER1, QTIMER3_ENCODER2
    };
    static const encoder_channel1_enum expected_a[] = {
        QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER2_CH1_C2,
        QTIMER2_ENCODER1_CH1_C3, QTIMER3_ENCODER2_CH1_B18
    };
    static const encoder_channel2_enum expected_b[] = {
        QTIMER1_ENCODER1_CH2_C1, QTIMER1_ENCODER2_CH2_C24,
        QTIMER2_ENCODER1_CH2_C25, QTIMER3_ENCODER2_CH2_B19
    };
    assert(encoder_init_calls < 4);
    assert(index == expected_index[encoder_init_calls]);
    assert(a == expected_a[encoder_init_calls] && b == expected_b[encoder_init_calls]);
    assert((int)index == (int)a / 2 && (int)index == (int)b / 2);
    encoder_init_calls++;
}

int16 encoder_get_count(encoder_index_enum index)
{
    return counts[index];
}

void encoder_clear_count(encoder_index_enum index)
{
    counts[index] = 0;
    clear_calls[index]++;
}

static void check_single_wheel(unsigned wheel, int command)
{
    int speeds[4] = {0};
    motor_speed_debug_snapshot_t snapshot;
    unsigned i;
    speeds[wheel] = command;
    motor_pwm(speeds[0], speeds[1], speeds[2], speeds[3]);
    motor_speed_debug_get_snapshot(&snapshot);
    for (i = 0; i < 4; ++i)
    {
        assert(duties[expected_pwm[i]] == (i == wheel ? (uint32)abs(command) : 0));
        if (i == wheel)
            assert(levels[expected_dir[i]] ==
                   (command < 0 ? 1 - expected_forward[i] : expected_forward[i]));
        assert(snapshot.final_pwm[i] == speeds[i]);
    }
}

static void check_feedback(int direction)
{
    unsigned tick;
    motor_speed_debug_snapshot_t snapshot;
    motor_speed_debug_reset();
    for (tick = 0; tick < 10; ++tick)
    {
        /* Physical wiring: encoder 1=DL, 2=DR, 3=UR, 4=UL. */
        counts[QTIMER1_ENCODER1] = (int16)(direction * 30);
        counts[QTIMER1_ENCODER2] = (int16)(direction * 40);
        counts[QTIMER2_ENCODER1] = (int16)(direction * -20);
        counts[QTIMER3_ENCODER2] = (int16)(direction * 10);
        encoder_get();
        assert(counts[QTIMER1_ENCODER1] == 0 && counts[QTIMER1_ENCODER2] == 0);
        assert(counts[QTIMER2_ENCODER1] == 0 && counts[QTIMER3_ENCODER2] == 0);
        motor_speed_debug_get_snapshot(&snapshot);
        for (unsigned wheel = 0; wheel < 4; ++wheel)
            assert(snapshot.raw_counts[wheel] == direction * (int)(10 * (wheel + 1)));
    }
    assert(up_L_all == direction * 10 && up_R_all == direction * 20);
    assert(down_L_all == direction * 30 && down_R_all == direction * 40);
    assert(encoders_average == direction * 35);
    for (unsigned wheel = 0; wheel < 4; ++wheel)
        assert(snapshot.cumulative_raw_counts[wheel] == direction * (int)(100 * (wheel + 1)));
}

static void check_closed_loop(unsigned wheel, int command)
{
    PIDInitStruct simple_pid = {1.0f, 0.0f, 0.0f, 4000.0f, 6000.0f, 1.0f};
    int targets[4] = {0};
    motor_speed_debug_snapshot_t snapshot;
    up_L_all = up_R_all = down_L_all = down_R_all = 0;
    PID_Init(&ULpid, &simple_pid);
    PID_Init(&URpid, &simple_pid);
    PID_Init(&DLpid, &simple_pid);
    PID_Init(&DRpid, &simple_pid);
    targets[wheel] = command;
    motor_control(targets);
    motor_speed_debug_get_snapshot(&snapshot);
    for (unsigned i = 0; i < 4; ++i)
    {
        assert(targets[i] == (i == wheel ? command : 0));
        assert(snapshot.target_counts[i] == targets[i]);
        assert(snapshot.pid_pwm[i] == targets[i]);
        if (i != wheel)
            assert(duties[expected_pwm[i]] == 0);
        else
        {
            assert(duties[expected_pwm[i]] > (uint32)abs(command));
            assert(levels[expected_dir[i]] ==
                   (command < 0 ? 1 - expected_forward[i] : expected_forward[i]));
        }
    }
}

static void check_pwm_test_mode(void)
{
    motor_speed_debug_snapshot_t snapshot;
    uint32 control_ticks;
    motor_init_calls = pwm_init_calls = encoder_init_calls = 0;
    app_control_init();
    assert(motor_pwm_test_enabled && !motor_run_enabled);
    for (unsigned i = 0; i < 4; ++i)
        assert(motor_test_pwm[i] == 0 && duties[expected_pwm[i]] == 0);
    motor_speed_debug_get_snapshot(&snapshot);
    control_ticks = snapshot.control_ticks;
    motor_run_enabled = true;
    for (unsigned wheel = 0; wheel < 4; ++wheel)
    {
        for (int direction = -1; direction <= 1; direction += 2)
        {
            for (unsigned i = 0; i < 4; ++i)
                motor_test_pwm[i] = i == wheel ? (int16)(direction * 100) : 0;
            for (unsigned tick = 0; tick < 5; ++tick)
            {
                /* Changing feedback must not adjust the PWM or run the PID. */
                counts[QTIMER1_ENCODER1] = (int16)(tick * 100);
                counts[QTIMER1_ENCODER2] = (int16)(-300 + (int)tick * 100);
                counts[QTIMER2_ENCODER1] = -500;
                counts[QTIMER3_ENCODER2] = 700;
                app_control_motor_tick_10ms();
                motor_speed_debug_get_snapshot(&snapshot);
                assert(snapshot.control_ticks == control_ticks);
                for (unsigned i = 0; i < 4; ++i)
                {
                    assert(duties[expected_pwm[i]] == (i == wheel ? 100 : 0));
                    assert(snapshot.final_pwm[i] == motor_test_pwm[i]);
                    if (i == wheel)
                        assert(levels[expected_dir[i]] ==
                               (direction < 0 ? 1 - expected_forward[i] : expected_forward[i]));
                }
            }
        }
    }
    motor_test_pwm[0] = INT16_MAX;
    motor_test_pwm[1] = INT16_MIN;
    app_control_motor_tick_10ms();
    assert(duties[expected_pwm[0]] == LIMIT_PWM_MAX);
    assert(duties[expected_pwm[1]] == (uint32)-LIMIT_PWM_MIN);

    motor_run_enabled = false;
    app_control_motor_tick_10ms();
    assert(abs(up_L_all) >= 5); /* Stop is immediate even while moving. */
    for (unsigned i = 0; i < 4; ++i)
        assert(duties[expected_pwm[i]] == 0);
    motor_speed_debug_get_snapshot(&snapshot);
    assert(snapshot.control_ticks == control_ticks);

    motor_pwm_test_enabled = false;
    motor_run_enabled = true;
    ULpid.fPre_Out = 1234;
    app_control_motor_tick_10ms();
    assert(!motor_run_enabled && ULpid.fPre_Out == 0);
    for (unsigned i = 0; i < 4; ++i)
        assert(duties[expected_pwm[i]] == 0);
    motor_run_enabled = true;
    motor_cmd_vx_cmps = 50;
    app_control_motor_tick_10ms();
    motor_speed_debug_get_snapshot(&snapshot);
    assert(snapshot.control_ticks == control_ticks + 1);

    motor_pwm_test_enabled = true;
    app_control_motor_tick_10ms();
    assert(!motor_run_enabled);
    for (unsigned i = 0; i < 4; ++i)
        assert(duties[expected_pwm[i]] == 0);
}

static void check_mixed_encoder_resolution(void)
{
    const float circumference = 0.11f * 3.1415926f;
    const float physical_counts[] = {20.0f, 20.0f, 20.0f, 10.0f};
    const float counts_per_turn[] = {2355.2f, 2355.2f, 2355.2f, 1177.6f};
    float command[3] = {100.0f, 0.0f, 0.0f};
    int targets[4];
    motor_speed_debug_snapshot_t snapshot;
    PIDInitStruct simple_pid = {1.0f, 0.0f, 0.0f, 4000.0f, 6000.0f, 1.0f};
    for (unsigned i = 0; i < 4; ++i)
    {
        assert(fabsf(motor_encoder_counts_per_revolution[i] - counts_per_turn[i]) < 0.001f);
        assert(fabsf(motor_encoder_counts_to_cmps(i, counts_per_turn[i]) - circumference * 10000.0f) < 0.001f);
        assert(fabsf(motor_reference_counts(i, physical_counts[i]) - 20.0f) < 0.001f);
    }
    assert(fabsf(motor_encoder_counts_to_cmps(0, 20.0f) - motor_encoder_counts_to_cmps(3, 10.0f)) < 0.001f);
    assert(fabsf(motor_encoder_counts_to_cmps(3, -10.0f) + motor_encoder_counts_to_cmps(0, 20.0f)) < 0.001f);
    assert(motor_encoder_counts_to_cmps(4, 1.0f) == 0.0f);
    Kinematics_Init();
    Kinematics_Inverse(command, targets);
    for (unsigned i = 0; i < 4; ++i)
        assert(targets[i] == 68); /* 1 m/s -> 68 reference counts / 10ms. */
    assert(MOTOR_RIGHT_START_DISTANCE_COUNTS == 341); /* 5 cm with 11 cm wheels. */
    for (unsigned tick = 0; tick < 10; ++tick)
    {
        counts[QTIMER1_ENCODER1] = 20;
        counts[QTIMER1_ENCODER2] = 10;
        counts[QTIMER2_ENCODER1] = -20;
        counts[QTIMER3_ENCODER2] = 20;
        encoder_get();
    }
    assert(up_L_all == 20 && up_R_all == 20 && down_L_all == 20 && down_R_all == 10);
    assert(encoders_average == 20);
    PID_Init(&ULpid, &simple_pid);
    PID_Init(&URpid, &simple_pid);
    PID_Init(&DLpid, &simple_pid);
    PID_Init(&DRpid, &simple_pid);
    for (unsigned i = 0; i < 4; ++i) targets[i] = 40;
    motor_control(targets);
    motor_speed_debug_get_snapshot(&snapshot);
    for (unsigned i = 0; i < 4; ++i)
    {
        assert(snapshot.pid_pwm[i] == 20); /* Same physical speed gives same PID error. */
        assert(snapshot.filtered_counts[i] == (int)physical_counts[i]);
        assert(fabsf(snapshot.wheel_speed_cmps[i] - snapshot.wheel_speed_cmps[0]) < 0.001f);
    }
    /* Equal lateral wheel speeds must end the 5cm launch window together,
     * despite DR producing half as many physical encoder counts. */
    const int right_targets[4] = {40, -40, -40, 40};
    int limited[4];
    up_L_all = 40; up_R_all = -40; down_L_all = -40; down_R_all = 20;
    motor_right_start_compensation_reset();
    for (unsigned tick = 0; tick < 9; ++tick)
    {
        motor_limit_right_start_forward_offset(right_targets, limited);
        for (unsigned i = 0; i < 4; ++i)
            assert(limited[i] == right_targets[i] - (tick < 8 ? 1 : 0));
    }
    puts("mixed encoder tests passed: 1024/512 lines, 2.3 ratio, 11cm wheels, equal-speed PID feedback");
}

static void position_tick(void)
{
    imu_tick(); imu_tick(); app_control_motor_tick_10ms();
}

static void assert_position_stopped(void)
{
    assert(!motor_run_enabled);
    assert(motor_cmd_vx_cmps == 0.0f && motor_cmd_vy_cmps == 0.0f && motor_cmd_omega_radps == 0.0f);
    for (unsigned i = 0; i < 4; ++i) assert(duties[expected_pwm[i]] == 0);
    assert(ULpid.fPre_Out == 0 && DRpid.fPre_Out == 0);
}

static void check_position_integration(void)
{
    position_output_t out;
    navigation_snapshot_t pose;
    memset(counts, 0, sizeof(counts));
    motor_position_enabled = true;
    motor_pwm_test_enabled = false;
    motor_position_goal = (position_goal_t){50.0f, 0.0f, 0.0f};
    motor_run_enabled = true;
    position_tick(); /* Changing modes must not start moving. */
    assert_position_stopped();
    motor_run_enabled = true;
    position_tick(); /* Previous AHRS generation fault is still latched. */
    app_control_get_position_snapshot(&out);
    assert(out.status == POSITION_NO_POSE);
    assert_position_stopped();
    app_navigation_request_reset(); position_tick();
    for (unsigned i = 0; i < 55; ++i) position_tick();
    app_navigation_get_snapshot(&pose);
    assert(pose.valid);
    assert_position_stopped(); /* Valid pose recovery doesn't restart Run. */
    motor_run_enabled = true;
    for (unsigned i = 0; i < 60; ++i) position_tick();
    app_control_get_position_snapshot(&out);
    assert(out.status == POSITION_MOVING && motor_cmd_vx_cmps > 0.0f);
    for (unsigned i = 0; i < 4; ++i) {
        assert(speed_encoder[i] > 0);
        assert(duties[expected_pwm[i]] > 0);
    }
    /* Replay equal physical motion toward X=50cm, DR at half count rate. */
    for (unsigned i = 0; i < 166; ++i) {
        counts[ENCODER_1] = 20; counts[ENCODER_2] = 10;
        counts[ENCODER_3] = -20; counts[ENCODER_4] = 20;
        position_tick();
    }
    app_navigation_get_snapshot(&pose);
    assert(fabsf(pose.x_m * 100.0f - 50.0f) < 2.0f);
    assert(motor_run_enabled); /* In tolerance but measured speed is not settled. */
    memset(counts, 0, sizeof(counts));
    for (unsigned i = 0; i < 100; ++i) position_tick();
    app_control_get_position_snapshot(&out);
    assert(out.status == POSITION_REACHED);
    assert_position_stopped();
    motor_position_goal = (position_goal_t){0, 0, 0};
    position_tick();
    app_control_get_position_snapshot(&out);
    assert(out.status == POSITION_IDLE); /* New target must not display old arrival. */
    assert_position_stopped(); /* Editing a target doesn't auto-start. */
    motor_run_enabled = true;
    for (unsigned i = 0; i < 40; ++i) position_tick();
    assert(motor_run_enabled && motor_cmd_vx_cmps < 0.0f);
    for (unsigned i = 0; i < 7; ++i) app_control_motor_tick_10ms();
    app_control_get_position_snapshot(&out);
    assert(out.status == POSITION_NO_POSE);
    assert_position_stopped();
    app_navigation_request_reset(); position_tick();
    for (unsigned i = 0; i < 55; ++i) position_tick();
    assert_position_stopped();
    motor_position_goal.x_cm = 50;
    motor_run_enabled = true;
    for (unsigned i = 0; i < 40; ++i) position_tick();
    motor_position_goal.yaw_deg = 90;
    fake_imu.gyro_dps[2] = 300.0f;
    for (unsigned i = 0; i < 30; ++i) {
        fake_imu.yaw_deg += 3.0f;
        position_tick();
    }
    fake_imu.gyro_dps[2] = 0;
    for (unsigned i = 0; i < 60; ++i) position_tick();
    assert(motor_run_enabled && fabsf(motor_cmd_vx_cmps) < 0.001f);
    assert(motor_cmd_vy_cmps < 0 && fabsf(motor_cmd_omega_radps) < 0.001f);
    /* The same world +X point is now a right-strafe body command at yaw 90. */
    assert(speed_encoder[0] > 0 && speed_encoder[1] < 0);
    assert(speed_encoder[2] < 0 && speed_encoder[3] > 0);
    motor_run_enabled = false; position_tick();
    assert_position_stopped();
    motor_run_enabled = true; position_tick();
    app_navigation_request_reset(); position_tick();
    assert_position_stopped(); /* Zero while running cancels the move immediately. */
    for (unsigned i = 0; i < 55; ++i) position_tick();
    motor_position_config.max_speed_cmps = NAN;
    motor_run_enabled = true; position_tick();
    app_control_get_position_snapshot(&out);
    assert(out.status == POSITION_BAD_CONFIG);
    assert_position_stopped();
    motor_position_config = (position_config_t)POSITION_CONFIG_DEFAULT;
    motor_position_enabled = false; position_tick();
    motor_cmd_vx_cmps = 20; motor_run_enabled = true; position_tick();
    assert(motor_run_enabled && speed_encoder[0] > 0); /* Manual Drive still works. */
    motor_position_enabled = true; motor_pwm_test_enabled = true;
    position_tick(); motor_run_enabled = true; position_tick();
    assert_position_stopped(); /* Conflicting modes can't actuate PWM. */
    motor_position_enabled = false; position_tick();
    motor_test_pwm[0] = 100; motor_test_pwm[1] = motor_test_pwm[2] = motor_test_pwm[3] = 0;
    motor_run_enabled = true; position_tick();
    assert(duties[expected_pwm[0]] == 100 && duties[expected_pwm[1]] == 0);
    motor_run_enabled = false; position_tick();
    puts("position integration passed: fusion to kinematics/PID/PWM, arrival, stale IMU, Zero, mode isolation, manual Drive");
}

int main(void)
{
    motor_speed_debug_snapshot_t snapshot;
    motor_init();
    encoder_init();
    assert(motor_init_calls == 4 && pwm_init_calls == 4 && encoder_init_calls == 4);
    for (unsigned wheel = 0; wheel < 4; ++wheel)
    {
        check_single_wheel(wheel, 100 * (int)(wheel + 1));
        check_single_wheel(wheel, -100 * (int)(wheel + 1));
    }
    motor_pwm(INT_MAX, INT_MIN, 10000, -10000);
    motor_speed_debug_get_snapshot(&snapshot);
    for (unsigned wheel = 0; wheel < 4; ++wheel)
    {
        int expected = wheel % 2 == 0 ? LIMIT_PWM_MAX : LIMIT_PWM_MIN;
        assert(duties[expected_pwm[wheel]] == (uint32)abs(expected));
        assert(snapshot.final_pwm[wheel] == expected);
    }
    motor_pwm(0, 0, 0, 0);
    for (unsigned wheel = 0; wheel < 4; ++wheel)
        assert(duties[expected_pwm[wheel]] == 0);
    check_feedback(1);
    check_feedback(-1);
    assert(clear_calls[QTIMER1_ENCODER1] == 20 && clear_calls[QTIMER1_ENCODER2] == 20);
    assert(clear_calls[QTIMER2_ENCODER1] == 20 && clear_calls[QTIMER3_ENCODER2] == 20);
    for (unsigned wheel = 0; wheel < 4; ++wheel)
    {
        check_closed_loop(wheel, 100);
        check_closed_loop(wheel, -100);
    }
    check_pwm_test_mode();
    check_mixed_encoder_resolution();
    check_navigation_integration();
    check_position_integration();
    puts("motor tests passed: mapping, PID routing, direct PWM, feedback independence, stop, mode switching");
    return 0;
}
