#ifndef IMU_NUMERIC_H
#define IMU_NUMERIC_H
#include <stdint.h>
#include <string.h>
/* ADS模板使用fastSingle浮点优化。对NaN/Inf的检查采用IEEE754位模式，
 * 避免依赖某些编译器在快速浮点模式下可能优化掉的isfinite比较。
 * TC264为32位IEEE754 float；memcpy避免别名访问和long位宽假设。 */
static int imu_finite(float value)
{
    uint32_t bits;
    typedef char float_size_check[(sizeof(float)==sizeof(uint32_t)) ? 1 : -1];
    (void)sizeof(float_size_check);
    memcpy(&bits,&value,sizeof(bits));
    return (bits & 0x7f800000u)!=0x7f800000u;
}
#endif
