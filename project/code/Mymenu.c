#include "Mymenu.h"

#include "Motor.h"
#include "PID_config.h"
#include "app_control.h"
#include "imu_attitude.h"
#include "imu_wifi_spi.h"
#include "menu.h"
#include "zf_common_font.h"
#include "zf_common_interrupt.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define MENU_COLUMNS        30
#define MENU_LINE_HEIGHT    16
#define MENU_VISIBLE_LINES  7
#define MENU_STEP_COUNT     5u

static Menu_Item g_root;
static Menu_Item *g_pointer;
static float g_steps[MENU_STEP_COUNT] = {0.01f, 0.1f, 1.0f, 10.0f, 100.0f};
static uint8_t g_step_index = 2u;
static volatile bool g_refresh_pending;
static bool g_imu_recalibrate;
static Menu_Item *g_encoder_folder;
static bool g_encoder_zero;
static uint8_t g_refresh_ticks;
static motor_speed_debug_snapshot_t g_motor_snapshot;
static int32 g_encoder_zero_counts[MOTOR_WHEEL_COUNT];

/* 240 pixels / 8 pixels per glyph = 30 characters. Padding clears old text. */
static void menu_show_line(uint16 y, const char *text)
{
    char bounded[MENU_COLUMNS + 1];
    snprintf(bounded, sizeof(bounded), "%-*.*s", MENU_COLUMNS, MENU_COLUMNS, text);
    ips200_show_string(0, y, bounded);
}

static void menu_motor_snapshot(void)
{
    uint32 primask = interrupt_global_disable();
    motor_speed_debug_get_snapshot(&g_motor_snapshot);
    interrupt_global_enable(primask);
}

static void menu_create(void)
{
    Menu_Item *pwm_test = Create_Menu_Folder_dynamic(&g_root, "PWM_Test");
    Menu_Item *drive = Create_Menu_Folder_dynamic(&g_root, "Drive");
    Menu_Item *encoder = Create_Menu_Folder_dynamic(&g_root, "Encoder");
    Menu_Item *imu = Create_Menu_Folder_dynamic(&g_root, "IMU");
    Menu_Item *sensor = Create_Menu_Folder_dynamic(&g_root, "Sensor");
    Menu_Item *wifi = Create_Menu_Folder_dynamic(&g_root, "WiFi");
    Menu_Item *pid = Create_Menu_Folder_dynamic(&g_root, "PID");
    Menu_Item *pid_ul;
    Menu_Item *pid_ur;
    Menu_Item *pid_dl;
    Menu_Item *pid_dr;

    g_encoder_folder = encoder;

    Create_Menu_File_dynamic(drive, "Run", (void *)&motor_run_enabled, bool_Box);
    Create_Menu_File_dynamic(drive, "Vx_cmps", (void *)&motor_cmd_vx_cmps, float_Box);
    Create_Menu_File_dynamic(drive, "Vy_cmps", (void *)&motor_cmd_vy_cmps, float_Box);
    Create_Menu_File_dynamic(drive, "Omega", (void *)&motor_cmd_omega_radps, float_Box);

    Create_Menu_File_dynamic(pwm_test, "OpenLoop", (void *)&motor_pwm_test_enabled, bool_Box);
    Create_Menu_File_dynamic(pwm_test, "Run", (void *)&motor_run_enabled, bool_Box);
    Create_Menu_File_dynamic(pwm_test, "UL_PWM", (void *)&motor_test_pwm[MOTOR_WHEEL_UL], int16_Box);
    Create_Menu_File_dynamic(pwm_test, "UR_PWM", (void *)&motor_test_pwm[MOTOR_WHEEL_UR], int16_Box);
    Create_Menu_File_dynamic(pwm_test, "DL_PWM", (void *)&motor_test_pwm[MOTOR_WHEEL_DL], int16_Box);
    Create_Menu_File_dynamic(pwm_test, "DR_PWM", (void *)&motor_test_pwm[MOTOR_WHEEL_DR], int16_Box);

    Create_Menu_Readonly_dynamic(encoder, "UL", &up_L_all, int16_Box);
    Create_Menu_Readonly_dynamic(encoder, "UR", &up_R_all, int16_Box);
    Create_Menu_Readonly_dynamic(encoder, "DL", &down_L_all, int16_Box);
    Create_Menu_Readonly_dynamic(encoder, "DR", &down_R_all, int16_Box);
    Create_Menu_File_dynamic(encoder, "ZeroTotal", &g_encoder_zero, bool_Box);

    Create_Menu_Readonly_dynamic(imu, "Status", (void *)&imu_attitude_status, int32_Box);
    Create_Menu_Readonly_dynamic(imu, "CalPct", (void *)&imu_calibration_percent, float_Box);
    Create_Menu_Readonly_dynamic(imu, "Roll", (void *)&imu_roll_deg, float_Box);
    Create_Menu_Readonly_dynamic(imu, "Pitch", (void *)&imu_pitch_deg, float_Box);
    Create_Menu_Readonly_dynamic(imu, "Yaw", (void *)&imu_yaw_deg, float_Box);
    Create_Menu_File_dynamic(imu, "Recal", &g_imu_recalibrate, bool_Box);

    Create_Menu_Readonly_dynamic(sensor, "Gx_dps", (void *)&imu_gyro_dps[0], float_Box);
    Create_Menu_Readonly_dynamic(sensor, "Gy_dps", (void *)&imu_gyro_dps[1], float_Box);
    Create_Menu_Readonly_dynamic(sensor, "Gz_dps", (void *)&imu_gyro_dps[2], float_Box);
    Create_Menu_Readonly_dynamic(sensor, "Ax_g", (void *)&imu_accel_g[0], float_Box);
    Create_Menu_Readonly_dynamic(sensor, "Ay_g", (void *)&imu_accel_g[1], float_Box);
    Create_Menu_Readonly_dynamic(sensor, "Az_g", (void *)&imu_accel_g[2], float_Box);

    Create_Menu_Readonly_dynamic(wifi, "Status", (void *)&imu_wifi_status, int32_Box);
    Create_Menu_Readonly_dynamic(wifi, "Packets", (void *)&imu_wifi_tx_packets, uint32_Box);
    Create_Menu_Readonly_dynamic(wifi, "Attempts", (void *)&imu_wifi_init_attempts, uint32_Box);
    Create_Menu_Readonly_dynamic(wifi, "LastErr", (void *)&imu_wifi_last_error, int32_Box);

    pid_ul = Create_Menu_Folder_dynamic(pid, "UL");
    pid_ur = Create_Menu_Folder_dynamic(pid, "UR");
    pid_dl = Create_Menu_Folder_dynamic(pid, "DL");
    pid_dr = Create_Menu_Folder_dynamic(pid, "DR");
    Create_Menu_File_dynamic(pid_ul, "Kp", &ULpid.fKp, float_Box);
    Create_Menu_File_dynamic(pid_ul, "Ki", &ULpid.fKi, float_Box);
    Create_Menu_File_dynamic(pid_ul, "Kd", &ULpid.fKd, float_Box);
    Create_Menu_File_dynamic(pid_ur, "Kp", &URpid.fKp, float_Box);
    Create_Menu_File_dynamic(pid_ur, "Ki", &URpid.fKi, float_Box);
    Create_Menu_File_dynamic(pid_ur, "Kd", &URpid.fKd, float_Box);
    Create_Menu_File_dynamic(pid_dl, "Kp", &DLpid.fKp, float_Box);
    Create_Menu_File_dynamic(pid_dl, "Ki", &DLpid.fKi, float_Box);
    Create_Menu_File_dynamic(pid_dl, "Kd", &DLpid.fKd, float_Box);
    Create_Menu_File_dynamic(pid_dr, "Kp", &DRpid.fKp, float_Box);
    Create_Menu_File_dynamic(pid_dr, "Ki", &DRpid.fKi, float_Box);
    Create_Menu_File_dynamic(pid_dr, "Kd", &DRpid.fKd, float_Box);
}

static void menu_format_value(const Menu_Item *item, char *buffer, size_t size)
{
    switch (item->kind)
    {
        case int32_Box:  snprintf(buffer, size, "%ld", (long)*(int32_t *)item->data); break;
        case uint32_Box: snprintf(buffer, size, "%lu", (unsigned long)*(uint32_t *)item->data); break;
        case int16_Box:  snprintf(buffer, size, "%d", (int)*(int16_t *)item->data); break;
        case uint16_Box: snprintf(buffer, size, "%u", (unsigned)*(uint16_t *)item->data); break;
        case int8_Box:   snprintf(buffer, size, "%d", (int)*(int8_t *)item->data); break;
        case uint8_Box:  snprintf(buffer, size, "%u", (unsigned)*(uint8_t *)item->data); break;
        case float_Box:  snprintf(buffer, size, "%.3f", (double)*(float *)item->data); break;
        case bool_Box:   snprintf(buffer, size, "%s", *(bool *)item->data ? "On" : "Off"); break;
        default:         snprintf(buffer, size, "[folder]"); break;
    }
}

static void menu_draw(void)
{
    Menu_Item *item = g_pointer->Father->First_Son;
    char line[MENU_COLUMNS + 1];
    char value[14];
    uint8_t row;

    snprintf(line, sizeof(line), "%-20s <%5.2f>", g_pointer->Father->name,
             (double)g_steps[g_step_index]);
    menu_show_line(0, line);

    if (g_pointer->Father == g_encoder_folder)
    {
        menu_show_line(MENU_LINE_HEIGHT, "  Wheel  Raw  Filt       Total");
        for (row = 0u; row < MOTOR_WHEEL_COUNT; ++row)
        {
            snprintf(line, sizeof(line), "%c%-2s %6d%6d%12ld",
                     item == g_pointer ? '>' : ' ', item->name,
                     g_motor_snapshot.raw_counts[row], g_motor_snapshot.filtered_counts[row],
                     (long)(g_motor_snapshot.cumulative_raw_counts[row] - g_encoder_zero_counts[row]));
            menu_show_line((uint16)((row + 2u) * MENU_LINE_HEIGHT), line);
            item = item->Next_Brother;
        }
        snprintf(line, sizeof(line), "%c%c%-12s %12s",
                 item == g_pointer ? '>' : ' ', item->selected ? '*' : ' ',
                 item->name, "Off");
        menu_show_line(6u * MENU_LINE_HEIGHT, line);
        menu_show_line(7u * MENU_LINE_HEIGHT, "Raw/Filt=count/10ms");
        snprintf(line, sizeof(line), "cm/s UL:%7.2f UR:%7.2f",
                 (double)g_motor_snapshot.wheel_speed_cmps[0],
                 (double)g_motor_snapshot.wheel_speed_cmps[1]);
        menu_show_line(128, line);
        snprintf(line, sizeof(line), "cm/s DL:%7.2f DR:%7.2f",
                 (double)g_motor_snapshot.wheel_speed_cmps[2],
                 (double)g_motor_snapshot.wheel_speed_cmps[3]);
        menu_show_line(144, line);
    }
    else
    {
        menu_show_line(128, "");
        menu_show_line(144, "");

        for (row = 0u; row < MENU_VISIBLE_LINES; ++row)
        {
            if (row < g_pointer->Father->sons)
            {
                menu_format_value(item, value, sizeof(value));
                snprintf(line, sizeof(line), "%c%c%-12s %12s",
                         item == g_pointer ? '>' : ' ',
                         item->selected ? '*' : ' ',
                         item->name,
                         value);
                item = item->Next_Brother;
            }
            else
            {
                snprintf(line, sizeof(line), "%30s", "");
            }
            menu_show_line((uint16)((row + 1u) * MENU_LINE_HEIGHT), line);
        }
    }

    snprintf(line, sizeof(line), "IMU:%ld Cal:%5.1f%%            ",
             (long)imu_attitude_status, (double)imu_calibration_percent);
    menu_show_line(160, line);
    snprintf(line, sizeof(line), "RPY:%7.2f %7.2f %7.2f",
             (double)imu_roll_deg, (double)imu_pitch_deg, (double)imu_yaw_deg);
    menu_show_line(176, line);
    snprintf(line, sizeof(line), "ENC:%5d %5d %5d %5d ",
             g_motor_snapshot.filtered_counts[0], g_motor_snapshot.filtered_counts[1],
             g_motor_snapshot.filtered_counts[2], g_motor_snapshot.filtered_counts[3]);
    menu_show_line(192, line);
    snprintf(line, sizeof(line), "PWM:%5d %5d %5d %5d ",
             g_motor_snapshot.final_pwm[0], g_motor_snapshot.final_pwm[1],
             g_motor_snapshot.final_pwm[2], g_motor_snapshot.final_pwm[3]);
    menu_show_line(208, line);
    snprintf(line, sizeof(line), "K1=enter K3=back K2/K4=move");
    menu_show_line(224, line);
}

static float menu_clamp(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}

static void menu_adjust(int direction)
{
    float delta = g_steps[g_step_index] * (float)direction;

    if (!g_pointer->editable)
    {
        return;
    }
    if (g_pointer->kind == bool_Box)
    {
        bool enabled = direction > 0;
        *(bool *)g_pointer->data = enabled;
        if (g_pointer->data == &g_imu_recalibrate && enabled)
        {
            imu_attitude_request_recalibration();
            g_imu_recalibrate = false;
        }
        else if (g_pointer->data == &g_encoder_zero && enabled)
        {
            /* Display baseline only: do not reset PWM, PID or wheel feedback. */
            menu_motor_snapshot();
            for (unsigned wheel = 0; wheel < MOTOR_WHEEL_COUNT; ++wheel)
                g_encoder_zero_counts[wheel] = g_motor_snapshot.cumulative_raw_counts[wheel];
            g_encoder_zero = false;
        }
        return;
    }
    if (g_pointer->kind == float_Box)
    {
        float value = *(float *)g_pointer->data + delta;
        if (g_pointer->data == (void *)&motor_cmd_vx_cmps ||
            g_pointer->data == (void *)&motor_cmd_vy_cmps)
        {
            value = menu_clamp(value, -300.0f, 300.0f);
        }
        else if (g_pointer->data == (void *)&motor_cmd_omega_radps)
        {
            value = menu_clamp(value, -20.0f, 20.0f);
        }
        else
        {
            value = menu_clamp(value, 0.0f, 1000.0f);
        }
        *(float *)g_pointer->data = value;
    }
    else if (g_pointer->kind == int16_Box)
    {
        /* PWM uses whole counts; fractional menu steps still move by one. */
        int step = (int)g_steps[g_step_index];
        int value;
        if (step < 1) step = 1;
        value = *(int16_t *)g_pointer->data + step * direction;
        *(int16_t *)g_pointer->data = (int16_t)Limit_int(LIMIT_PWM_MIN, value, LIMIT_PWM_MAX);
    }
}

void Menu_Init(void)
{
    ips200_set_dir(IPS200_PORTAIT);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(IPS200_TYPE_SPI);
    ips200_clear();
    key_init(20u);

    memset(&g_root, 0, sizeof(g_root));
    g_root.name = "MCAR";
    g_root.kind = MENU_Folder;
    menu_create();
    g_pointer = g_root.First_Son;
    All_Folder_Menu_Init(&g_root);
    g_refresh_ticks = 0;
    memset(g_encoder_zero_counts, 0, sizeof(g_encoder_zero_counts));
    g_encoder_zero = false;
    g_refresh_pending = true;
}

void Menu_Tick_20ms(void)
{
    if (++g_refresh_ticks >= 5u)
    {
        g_refresh_ticks = 0;
        g_refresh_pending = true;
    }
}

void Menu_Show(void)
{
    if (!g_refresh_pending)
    {
        return;
    }
    g_refresh_pending = false;
    menu_motor_snapshot();
    menu_draw();
}

void Menu_Switch(void)
{
    key_state_enum enter = key_get_state(KEY_1);
    key_state_enum up = key_get_state(KEY_2);
    key_state_enum back = key_get_state(KEY_3);
    key_state_enum down = key_get_state(KEY_4);
    bool repeat_numeric = g_pointer->selected && g_pointer->editable &&
                          (g_pointer->kind == float_Box || g_pointer->kind == int16_Box);
    bool up_repeat = up == KEY_LONG_PRESS || up == KEY_REPEAT_PRESS;
    bool down_repeat = down == KEY_LONG_PRESS || down == KEY_REPEAT_PRESS;

    if (enter != KEY_SHORT_PRESS && up != KEY_SHORT_PRESS &&
        back != KEY_SHORT_PRESS && down != KEY_SHORT_PRESS &&
        !up_repeat && !down_repeat)
    {
        return;
    }

    g_refresh_pending = true;

    if (up == KEY_SHORT_PRESS || (repeat_numeric && up_repeat))
    {
        if (g_pointer->selected) menu_adjust(1);
        else g_pointer = g_pointer->Last_Brother;
    }
    else if (down == KEY_SHORT_PRESS || (repeat_numeric && down_repeat))
    {
        if (g_pointer->selected) menu_adjust(-1);
        else g_pointer = g_pointer->Next_Brother;
    }
    else if (enter == KEY_SHORT_PRESS)
    {
        if (g_pointer->kind == MENU_Folder && g_pointer->First_Son != NULL)
            g_pointer = g_pointer->First_Son;
        else if (g_pointer->editable && !g_pointer->selected)
            g_pointer->selected = true;
        else if (g_pointer->editable)
            g_step_index = (uint8_t)((g_step_index + MENU_STEP_COUNT - 1u) % MENU_STEP_COUNT);
    }
    else if (back == KEY_SHORT_PRESS)
    {
        if (g_pointer->selected)
            g_pointer->selected = false;
        else if (g_pointer->Father->Father != NULL)
            g_pointer = g_pointer->Father;
    }

    key_clear_all_state();
}
