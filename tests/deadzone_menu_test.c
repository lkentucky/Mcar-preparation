#define main legacy_menu_test_main
#include "menu_display_test.c"
#undef main
int main(void)
{
    Menu_Init();Menu_Show();press(KEY_3);enter_root_folder("Drive");
    for(unsigned i=0;i<4;++i) press(KEY_4);
    for(unsigned i=0;i<8;++i) {
        volatile int *value=(i%2?motor_deadzone_rev:motor_deadzone_fwd)+i/2;
        *value=10;press(KEY_1);press(KEY_2);assert(*value==11);
        press(KEY_4);assert(*value==10);
        *value=0;press(KEY_4);assert(*value==0);
        *value=2000;press(KEY_2);assert(*value==2000);
        press(KEY_3);press(KEY_4);
    }
    puts("deadzone Drive menu passed: 8 integer bindings, 0..2000 limits, display bounds");return 0;
}
