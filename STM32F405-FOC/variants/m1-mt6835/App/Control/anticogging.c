#include "anticogging.h"
#include <math.h>
#include <string.h>

cog_table_t cog_table;
static cog_table_t candidate;
static volatile unsigned uploaded;
static volatile bool valid, enabled, uploading;
static volatile float peak, applied;
static float activation;

static bool bound(const foc_calibration_t *cal)
{
    return foc.calibrated && memcmp(cal, &foc.calibration, sizeof *cal) == 0;
}
bool cog_valid(void) { return valid && bound(&cog_table.calibration); }
unsigned cog_flags(void) { return (unsigned)cog_valid() | ((unsigned)enabled << 1) | ((unsigned)uploading << 2); }
unsigned cog_uploaded(void) { return uploaded; }
float cog_peak(void) { return peak; }
float cog_applied(void) { return applied; }
void cog_stop(void) { applied = activation = 0.0f; }
void cog_cancel(void) { uploading = false; uploaded = 0; }
void cog_reset(void) { valid = enabled = uploading = false; uploaded = 0; peak = applied = activation = 0; cog_table.gain = 0.5f; }
bool cog_begin(void)
{
    if (foc.state != FOC_IDLE || !foc.calibrated || enabled) return false;
    candidate.calibration = foc.calibration;
    candidate.gain = 0.5f;
    uploaded = 0; uploading = true;
    return true;
}
bool cog_point(unsigned index, float amps)
{
    if (foc.state != FOC_IDLE || !uploading || !bound(&candidate.calibration) ||
        index != uploaded || index >= COG_POINTS || !isfinite(amps) || fabsf(amps) > COG_MAX_AMPS) return false;
    candidate.map[uploaded++] = amps;
    return true;
}
/* The disabled active buffer can be prepared without masking the current ISR.
 * The foreground is the only writer; enable is a separate later command. */
bool cog_commit(void)
{
    if (foc.state != FOC_IDLE || enabled || !uploading || uploaded != COG_POINTS || !bound(&candidate.calibration)) return false;
    float mean = 0, maximum = 0;
    for (unsigned i = 0; i < COG_POINTS; ++i) mean += candidate.map[i] / COG_POINTS;
    for (unsigned i = 0; i < COG_POINTS; ++i) maximum = fmaxf(maximum, fabsf(candidate.map[i] - mean));
    if (maximum > COG_MAX_AMPS || maximum < 0.005f) return false;
    for (unsigned i = 0; i < COG_POINTS; ++i) candidate.map[i] -= mean;
    valid = false;
    cog_table = candidate;
    peak = maximum; applied = 0; uploading = false; valid = true;
    return true;
}
bool cog_enable(bool on)
{
    if (on && (!cog_valid() || uploading)) return false;
    enabled = on;
    if (!on) cog_stop();
    return true;
}
bool cog_gain(float gain)
{
    if (foc.state != FOC_IDLE || enabled || !isfinite(gain) || gain < 0 || gain > 1) return false;
    cog_table.gain = gain; return true;
}
bool cog_restore(const cog_table_t *table)
{
    if (!bound(&table->calibration) || !isfinite(table->gain) || table->gain < 0 || table->gain > 1) return false;
    float maximum = 0, mean = 0;
    for (unsigned i = 0; i < COG_POINTS; ++i) {
        if (!isfinite(table->map[i]) || fabsf(table->map[i]) > COG_MAX_AMPS) return false;
        maximum = fmaxf(maximum, fabsf(table->map[i])); mean += table->map[i] / COG_POINTS;
    }
    if (maximum < .005f || fabsf(mean) > .01f) return false;
    cog_table = *table; peak = maximum; valid = true; enabled = false; applied = 0;
    return true;
}
float cog_output(float degrees, float rpm, float dt)
{
    if (!enabled || !cog_valid() || !isfinite(degrees) || !isfinite(rpm)) { cog_stop(); return 0; }
    float angle = degrees - 360.0f * floorf(degrees / 360.0f);
    float index = angle * (COG_POINTS / 360.0f);
    unsigned a = (unsigned)index;
    if (a >= COG_POINTS) a = 0;
    unsigned b = a + 1 == COG_POINTS ? 0 : a + 1;
    float value = cog_table.map[a] + (index - (float)a) * (cog_table.map[b] - cog_table.map[a]);
    /* Full low-speed compensation, taper out before fast reversals. Ramp the
     * activation for 50 ms, not the spatial waveform: a per-sample Iq slew
     * introduces phase lag precisely where rapid cogging cancellation matters. */
    float fade = fminf(1, fmaxf(0, (120.0f - fabsf(rpm)) / 90.0f));
    activation=fminf(1.0f,activation+20.0f*dt);
    applied=value*cog_table.gain*fade*activation;
    return applied;
}
