#define main legacy_wifi_test_main
#include "wifi_telemetry_test.c"
#undef main
int main(void)
{
    wifispi_telemetry_init(); command("STREAM 0\n");
    command("GOTO 100 50 -25 45\n");
    assert(has_reply("MCAR NAV 100 50 -25 45\n") && remote_goals==1);
    assert(motor_position_goal.x_cm==50 && motor_position_goal.y_cm==-25 && motor_run_enabled);
    command("GOTO 100 50 -25 45\n"); assert(remote_goals==1);
    command("GOTO 100 51 -25 45\n"); assert(has_reply("MCAR NAVERR 100 stale_request\n"));
    command("STOP 101\n"); assert(!motor_run_enabled && remote_stops==1 && has_reply("MCAR STOP 101\n"));
    command("GOTO 100 50 -25 45\n"); assert(remote_goals==1 && !motor_run_enabled);
    command("GOTO 102 nan 0 0\n"); assert(remote_goals==1 && !motor_run_enabled);
    command("GOTO 102 10001 0 0\n"); assert(remote_goals==1);
    command("GOTO 102 1 2 0 junk\n"); assert(remote_goals==1);
    command("GOTO 4294967296 1 2 0\n"); assert(remote_goals==1);
    nav.valid=false;
    command("GOTO 102 1 2 0\n"); assert(remote_goals==1 && has_reply("MCAR NAVERR 102 pose_invalid\n"));
    nav.valid=true;
    command("GOTO 102 1 2 0\n"); assert(remote_goals==2);
    command("STOP 101\n"); assert(!motor_run_enabled);
    command("GOTO 101 4 5 0\n"); assert(remote_goals==2 && !motor_run_enabled);
    puts("click navigation transport passed: atomic goal, acknowledgements, duplicate/stale rejection, STOP, invalid pose/range");
    return 0;
}
