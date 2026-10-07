#ifndef MOTOR_TEST_GPIO_H
#define MOTOR_TEST_GPIO_H

#include "zf_common_typedef.h"

/* GPIO values match the library; PWM/encoder enums use the real headers. */
typedef enum { C4 = 68, C20 = 84, C26 = 90, C27 = 91, C31 = 95,
               D0 = 96, D1 = 97, D12 = 108, D13 = 109 } gpio_pin_enum;
typedef enum { GPI = 0, GPO = 1 } gpio_dir_enum;
enum { GPIO_LOW = 0, GPIO_HIGH = 1, GPO_PUSH_PULL = 0, GPI_PULL_UP = 1 };

void gpio_set_level(gpio_pin_enum pin, uint8 level);
#define gpio_low(pin) gpio_set_level((pin), GPIO_LOW)
#define gpio_high(pin) gpio_set_level((pin), GPIO_HIGH)
void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir, uint8 level, uint32 config);
uint8 gpio_get_level(gpio_pin_enum pin);

#endif
