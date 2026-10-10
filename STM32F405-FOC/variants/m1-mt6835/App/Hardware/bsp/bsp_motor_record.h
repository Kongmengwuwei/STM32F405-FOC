#ifndef APP_BSP_MOTOR_RECORD_H
#define APP_BSP_MOTOR_RECORD_H
#include "foc.h"
#include "foc_profile.h"
#include <stddef.h>
#include <string.h>

/* On-flash ABI: sector 11, magic committed last. In v3, poles packs
 * port/motor/encoder/installation identity and pole pairs. */
typedef struct { uint32_t version, poles; foc_calibration_t cal; uint32_t checksum, magic; } record_t;
static inline uint32_t checksum(const record_t *record)
{
    uint32_t hash = 2166136261u;
    const uint8_t *p = (const uint8_t *)record;
    for (unsigned i = 0; i < offsetof(record_t, checksum); ++i) hash = (hash ^ p[i]) * 16777619u;
    return hash;
}
static inline bool record_valid(const record_t *r)
{
    bool profile_match = r->version == 3u &&
        r->poles == ((FOC_CALIBRATION_ID << 16) | FOC_POLE_PAIRS);
    return r->magic == 0x464f4331u && profile_match &&
        r->checksum == checksum(r) && (r->cal.direction == 1 || r->cal.direction == -1) &&
        r->cal.zero >= 0.0f && r->cal.zero < 6.2831853072f;
}
/* Optional extension at +64; the existing v3 angle record remains readable.
 * Bind electrical tuning to the exact angle calibration and installation. */
typedef struct {
    uint32_t version, profile;
    foc_calibration_t cal;
    motor_parameters_t parameters;
    uint32_t checksum, magic;
} tuning_record_t;
static inline uint32_t tuning_checksum(const tuning_record_t *r)
{
    uint32_t hash=2166136261u;
    const uint8_t *p=(const uint8_t *)r;
    for(unsigned i=0;i<offsetof(tuning_record_t,checksum);++i)hash=(hash^p[i])*16777619u;
    return hash;
}
static inline bool tuning_record_valid(const tuning_record_t *r,const foc_calibration_t *cal)
{
    return r->version==1u && r->profile==((FOC_CALIBRATION_ID<<16)|FOC_POLE_PAIRS) &&
        r->magic==0x524c5049u && r->checksum==tuning_checksum(r) &&
        memcmp(&r->cal,cal,sizeof *cal)==0 && motor_parameters_valid(&r->parameters,(float)FOC_SAMPLE_HZ);
}
#endif
