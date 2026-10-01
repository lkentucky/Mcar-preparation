#ifndef HOST_TEST_INTERRUPT_H
#define HOST_TEST_INTERRUPT_H
#include "zf_common_typedef.h"
uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 primask);
#endif
