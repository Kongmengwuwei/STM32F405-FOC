#ifndef APP_BSP_MOTOR_RECORD_H
#define APP_BSP_MOTOR_RECORD_H
#include "foc.h"
#include "anticogging.h"
#include "foc_profile.h"
#include <stddef.h>
#include <string.h>

#define FOC_RECORD_ADDRESS 0x080e0000u
#define FOC_TUNING_ADDRESS (FOC_RECORD_ADDRESS + 64u)
#define FOC_COG_ADDRESS (FOC_RECORD_ADDRESS + 128u)

static inline uint32_t record_checksum_bytes(const void *data, size_t size)
{
    uint32_t hash = 2166136261u;
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < size; ++i) hash = (hash ^ p[i]) * 16777619u;
    return hash;
}

/* On-flash ABI: sector 11, magic committed last. v3 uses configured poles;
 * v4 accepts detected poles (1..64) with the same installation identity. */
typedef struct { float zero; int32_t direction; } flash_angle_t;
typedef struct { uint32_t version, poles; flash_angle_t cal; uint32_t checksum, magic; } record_t;
_Static_assert(sizeof(record_t)==24u,"Keep legacy Flash prefix ABI");
static inline uint32_t checksum(const record_t *record)
{
    return record_checksum_bytes(record, offsetof(record_t, checksum));
}
static inline bool record_valid(const record_t *r)
{
    bool profile_match = (r->version == 3u &&
        r->poles == ((FOC_CALIBRATION_ID << 16) | FOC_POLE_PAIRS)) ||
        (r->version==4u && (r->poles>>16)==FOC_CALIBRATION_ID &&
         (r->poles&0xffffu)>=1u && (r->poles&0xffffu)<=64u);
    return r->magic == 0x464f4331u && profile_match &&
        r->checksum == checksum(r) && (r->cal.direction == 1 || r->cal.direction == -1) &&
        r->cal.zero >= 0.0f && r->cal.zero < 6.2831853072f;
}
/* Optional extension at +64; existing v3/v1 records remain readable.
 * Bind electrical tuning to the exact angle calibration and installation. */
typedef struct {
    uint32_t version, profile;
    flash_angle_t cal;
    motor_parameters_t parameters;
    uint32_t checksum, magic;
} tuning_record_t;
_Static_assert(sizeof(tuning_record_t)==52u,"Keep tuning extension ABI");
static inline uint32_t tuning_checksum(const tuning_record_t *r)
{
    return record_checksum_bytes(r, offsetof(tuning_record_t, checksum));
}
static inline bool tuning_record_valid(const tuning_record_t *r,const foc_calibration_t *cal)
{
    return r->version==1u && cal->pole_pairs>=1u && cal->pole_pairs<=64u &&
        r->profile==((FOC_CALIBRATION_ID<<16)|cal->pole_pairs) &&
        r->magic==0x524c5049u && r->checksum==tuning_checksum(r) &&
        memcmp(&r->cal,cal,sizeof r->cal)==0 && motor_parameters_valid(&r->parameters,(float)FOC_SAMPLE_HZ);
}
/* Optional anticogging extension at +128, preserving both legacy prefixes. */
typedef struct {
    uint32_t version, profile, points;
    cog_table_t table;
    uint32_t checksum, magic;
} cog_record_t;
_Static_assert(sizeof(cog_record_t)==2916u,"Keep anticogging extension ABI");
_Static_assert(offsetof(record_t,magic)==sizeof(record_t)-4u,"Angle magic last");
_Static_assert(offsetof(tuning_record_t,magic)==sizeof(tuning_record_t)-4u,"Tuning magic last");
_Static_assert(offsetof(cog_record_t,magic)==sizeof(cog_record_t)-4u,"Cog magic last");
static inline uint32_t cog_checksum(const cog_record_t *r)
{
    return record_checksum_bytes(r, offsetof(cog_record_t, checksum));
}
static inline bool cog_record_valid(const cog_record_t *r,const foc_calibration_t *cal)
{
    return r->version==1u && r->points==COG_POINTS &&
        r->profile==((FOC_CALIBRATION_ID<<16)|cal->pole_pairs) &&
        memcmp(&r->table.calibration,cal,sizeof *cal)==0 &&
        r->magic==0x434f4731u && r->checksum==cog_checksum(r);
}
#endif
