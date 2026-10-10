#ifndef APP_ANTICOGGING_H
#define APP_ANTICOGGING_H
#include "foc.h"

#define COG_POINTS 720u
#define COG_MAX_AMPS 1.5f
/* Indexed by absolute mechanical encoder angle, independent of control_zero.
 * Foreground upload is ordered and committed only when complete, gates off. */
typedef struct {
    foc_calibration_t calibration;
    float map[COG_POINTS];
    float gain;
} cog_table_t;
extern cog_table_t cog_table;
void cog_reset(void);
bool cog_begin(void);
bool cog_point(unsigned index, float amps);
bool cog_commit(void); /* Foreground, not under an interrupt mask. */
void cog_cancel(void);
bool cog_enable(bool enabled);
bool cog_gain(float gain);
bool cog_valid(void);
unsigned cog_flags(void);
unsigned cog_uploaded(void);
float cog_peak(void);
float cog_output(float absolute_deg, float reference_rpm, float dt);
float cog_applied(void);
void cog_stop(void);
bool cog_restore(const cog_table_t *table);
#endif
