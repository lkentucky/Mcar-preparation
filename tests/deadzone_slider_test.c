#define main legacy_wifi_test_main
#include "wifi_telemetry_test.c"
#undef main
int main(void)
{
    const char *names[]={"ul_deadzone_fwd","ul_deadzone_rev","ur_deadzone_fwd","ur_deadzone_rev",
                         "dl_deadzone_fwd","dl_deadzone_rev","dr_deadzone_fwd","dr_deadzone_rev"};
    wifispi_telemetry_init();command("STREAM 0\n");
    motor_run_enabled=true;
    for(unsigned i=0;i<8;++i) {
        char request[100],reply[100];
        volatile int *value=(i%2?motor_deadzone_rev:motor_deadzone_fwd)+i/2;
        snprintf(request,sizeof(request),"[\"slider\",\"%s\",%u]",names[i],600+i);
        snprintf(reply,sizeof(reply),"MCAR SLIDER %s %u\n",names[i],600+i);
        command(request);assert(*value==(int)(600+i) && has_reply(reply));
        snprintf(request,sizeof(request),"[slider,%s,2.5]",names[i]);command(request);
        assert(*value==(int)(600+i) && has_reply("MCAR ERR parameter requires integer\n"));
        snprintf(request,sizeof(request),"[slider,%s,-1]",names[i]);command(request);
        assert(*value==(int)(600+i) && has_reply("MCAR ERR parameter out of range\n"));
        snprintf(request,sizeof(request),"[slider,%s,2001]",names[i]);command(request);
        assert(*value==(int)(600+i));
        snprintf(request,sizeof(request),"[slider,%s,0]",names[i]);command(request);assert(*value==0);
        snprintf(request,sizeof(request),"[slider,%s,2000]",names[i]);command(request);assert(*value==2000);
    }
    assert(motor_run_enabled && motor_position_enabled && !motor_pwm_test_enabled);
    command("SUB ul_deadzone_fwd,dr_deadzone_rev\nSTREAM 1\n");
    assert(last_value(0,2)==2000 && last_value(1,2)==2000);
    puts("deadzone slider passed: 8 bindings, live update, integer/range checks, subscription");return 0;
}
