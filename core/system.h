#ifndef INC_SYSTEM_H
#define INC_SYSTEM_H

#include "common-defines.h"

void system_setup(void);
uint64_t system_get_ticks(void);
void system_delay(uint64_t milliseconds);
void system_teardown(void);

#endif/* INC_SYSTEM_H */
