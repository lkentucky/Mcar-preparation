#ifndef HOST_TEST_CLOCK_H
#define HOST_TEST_CLOCK_H
#include "zf_common_typedef.h"

/* Only the CMSIS registers used by imu.c; no host hardware access. */
typedef struct { uint32 CYCCNT, CTRL, LAR; } host_dwt_t;
typedef struct { uint32 DEMCR; } host_core_debug_t;
extern host_dwt_t host_dwt;
extern host_core_debug_t host_core_debug;
extern uint32 system_clock;
#define DWT (&host_dwt)
#define CoreDebug (&host_core_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1u
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#endif
