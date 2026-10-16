#define main legacy_flash_test_main
#include "flash_route_test.c"
#undef main
int main(void)
{
    const int fwd[4]={MOTOR_UL_DEADZONE_FWD,MOTOR_UR_DEADZONE_FWD,MOTOR_DL_DEADZONE_FWD,MOTOR_DR_DEADZONE_FWD};
    const int rev[4]={MOTOR_UL_DEADZONE_REV,MOTOR_UR_DEADZONE_REV,MOTOR_DL_DEADZONE_REV,MOTOR_DR_DEADZONE_REV};
    menu_flash_config_t config, before;
    uint32 saved[FLASH_DATA_BUFFER_SIZE];
    for(unsigned i=0;i<4;++i) {motor_deadzone_fwd[i]=400+i*10;motor_deadzone_rev[i]=600+i*10;}
    assert(menu_flash_save_current());
    assert(storage[1]==5 && last_write_length==72);
    memcpy(saved,storage,sizeof(saved));
    for(unsigned i=0;i<4;++i) motor_deadzone_fwd[i]=motor_deadzone_rev[i]=0;
    assert(menu_flash_load_current());
    for(unsigned i=0;i<4;++i) {assert(motor_deadzone_fwd[i]==400+(int)i*10);assert(motor_deadzone_rev[i]==600+(int)i*10);}
    assert(!motor_run_enabled && !route_run_flag);
    assert(Data_load_from_flash(&config));before=config;
    unsigned old_writes=writes;
    config.wheel_deadzone_fwd[0]=-1;assert(!Data_save_to_flash(&config));
    config=before;config.wheel_deadzone_rev[3]=2001;assert(!Data_save_to_flash(&config));
    assert(writes==old_writes);
    config=before;config.wheel_deadzone_fwd[0]=0;config.wheel_deadzone_rev[3]=2000;
    assert(Data_save_to_flash(&config));assert(menu_flash_load_current());
    assert(motor_deadzone_fwd[0]==0 && motor_deadzone_rev[3]==2000);
    /* A corrupted word must not partly replace any live parameter. */
    storage[63]^=1;assert(!menu_flash_load_current());
    assert(motor_deadzone_rev[3]==2000);
    memcpy(storage,saved,sizeof(saved));storage[63]=0xffffffffu;checksum(71);
    assert(!Data_load_from_flash(&config));
    for(unsigned version=1;version<=4;++version) {
        memcpy(storage,saved,sizeof(saved));storage[1]=version;
        checksum(version==1?27:version==2?52:version==3?62:63);
        assert(menu_flash_load_current());
        for(unsigned i=0;i<4;++i) {assert(motor_deadzone_fwd[i]==fwd[i]);assert(motor_deadzone_rev[i]==rev[i]);}
        assert(!motor_run_enabled && !route_run_flag);
        assert(menu_flash_save_current());assert(storage[1]==5);
    }
    fail_write=1;assert(!menu_flash_save_current());fail_write=0;
    assert(!motor_run_enabled && !route_run_flag);
    puts("Flash deadzones passed: 8 values, bounds, corruption, v1-v4 migration, stopped boot, write failure");
    motor_position_config.max_accel_cmps2=123;
    assert(menu_flash_save_current() && menu_flash_save_status==1 && menu_flash_stored_accel==123);
    memcpy(stale_data,storage,sizeof(storage));
    motor_position_config.max_accel_cmps2=40;
    assert(menu_flash_load_current() && menu_flash_load_status==1);
    assert(motor_position_config.max_accel_cmps2==123 && menu_flash_stored_accel==123);
    /* A valid OLD archive is not proof that the NEW write succeeded. */
    motor_position_config.max_accel_cmps2=156;stale_read=1;
    assert(!menu_flash_save_current() && menu_flash_save_status==-4);
    stale_read=0;
    assert(menu_flash_save_current() && menu_flash_stored_accel==156);
    assert(Data_load_from_flash(&config));config.max_accel_cmps2=0;
    assert(!Data_save_to_flash(&config) && menu_flash_save_status==-1 && menu_flash_bad_word==19);
    puts("acceleration persistence passed: non-default reboot restore, stale-read detection, field-specific failure");
    return 0;
}
