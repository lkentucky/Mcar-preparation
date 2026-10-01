/* Actual key scanner with simulated GPIO, scanned every 20 ms. */
#include "zf_device_key.h"
#include <assert.h>
#include <stdio.h>

static uint8 levels[128];
static const gpio_pin_enum pins[] = KEY_LIST;

void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8 level, uint32 config)
{
    assert(dir == GPI && config == GPI_PULL_UP);
    levels[pin] = level;
}

uint8 gpio_get_level(gpio_pin_enum pin) { return levels[pin]; }

static void scans(unsigned n) { while (n--) key_scanner(); }

static void check_repeat(key_index_enum key)
{
    levels[pins[key]] = GPIO_LOW;
    scans(49);
    assert(key_get_state(key) == KEY_RELEASE);
    scans(1);
    assert(key_get_state(key) == KEY_LONG_PRESS);
    key_clear_state(key);
    for (unsigned repeat = 0; repeat < 50; ++repeat)
    {
        scans(3);
        assert(key_get_state(key) == KEY_RELEASE);
        scans(1);
        assert(key_get_state(key) == KEY_REPEAT_PRESS);
        key_clear_state(key);
    }
    levels[pins[key]] = GPIO_HIGH;
    scans(20);
    assert(key_get_state(key) == KEY_RELEASE); /* No extra short event on release. */
}

int main(void)
{
    key_init(20);
    levels[pins[KEY_2]] = GPIO_LOW;
    scans(2);
    levels[pins[KEY_2]] = GPIO_HIGH;
    scans(20);
    assert(key_get_state(KEY_2) == KEY_SHORT_PRESS); /* Latched until consumed. */
    key_clear_state(KEY_2);
    scans(10);
    assert(key_get_state(KEY_2) == KEY_RELEASE);
    check_repeat(KEY_2);
    check_repeat(KEY_4);
    /* A slow consumer gets one pending event, never a queued burst. */
    levels[pins[KEY_2]] = GPIO_LOW;
    scans(1000);
    assert(key_get_state(KEY_2) == KEY_LONG_PRESS);
    key_clear_state(KEY_2);
    scans(4);
    assert(key_get_state(KEY_2) == KEY_REPEAT_PRESS);
    levels[pins[KEY_2]] = GPIO_HIGH;
    key_clear_all_state();
    scans(10);
    assert(key_get_state(KEY_2) == KEY_RELEASE);
    key_init(20);
    assert(key_get_state(KEY_2) == KEY_RELEASE);
    puts("key tests passed: short latch, 1s hold, 80ms repeat, release, no queued burst");
    return 0;
}
