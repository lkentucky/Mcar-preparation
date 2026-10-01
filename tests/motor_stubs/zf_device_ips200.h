#ifndef HOST_TEST_IPS200_H
#define HOST_TEST_IPS200_H
#include "zf_common_typedef.h"
typedef enum { IPS200_TYPE_SPI, IPS200_TYPE_PARALLEL8 } ips200_type_enum;
typedef enum { IPS200_PORTAIT } ips200_dir_enum;
typedef enum { IPS200_6X8_FONT, IPS200_8X16_FONT } ips200_font_size_enum;
void ips200_set_dir(ips200_dir_enum dir);
void ips200_set_font(ips200_font_size_enum font);
void ips200_set_color(uint16 pen, uint16 background);
void ips200_init(ips200_type_enum type);
void ips200_clear(void);
void ips200_show_string(uint16 x, uint16 y, const char text[]);
#endif
