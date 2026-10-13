#ifndef LEG_GUARD_H
#define LEG_GUARD_H
#include <stdbool.h>
#include <stdint.h>
#define LEG_GEAR_RATIO 7.0f
#ifndef LEG_TARGET_DEG
#define LEG_TARGET_DEG 2.5f
#endif
#ifndef LEG_TRAVEL_DEG
#define LEG_TRAVEL_DEG 3.8f
#endif
#define LEG_CURRENT_MAX 9.0f
#define LEG_MEASURED_CURRENT_MAX 9.5f
bool leg_ready(unsigned);
float leg_travel(unsigned);
uint32_t leg_observe(unsigned, float, uint32_t);
bool leg_target_valid(unsigned, float, float);
float leg_current_limit(void);
bool leg_set_current_limit(float);
#endif
