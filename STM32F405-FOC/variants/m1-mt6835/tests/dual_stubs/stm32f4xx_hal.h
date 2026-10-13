#ifndef TEST_DUAL_HAL_H
#define TEST_DUAL_HAL_H
#include <stdint.h>
static inline uint32_t __get_PRIMASK(void) { return 0u; }
static inline void __disable_irq(void) {}
static inline void __set_PRIMASK(uint32_t key) { (void)key; }
static inline void __DMB(void) {}
#endif
