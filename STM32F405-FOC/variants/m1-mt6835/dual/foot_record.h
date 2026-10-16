#ifndef FOOT_RECORD_H
#define FOOT_RECORD_H
#include "fivebar.h"
/* One explicit save, never a write on boot or periodic target refresh. */
bool foot_record_load(fb_geometry_t *);
bool foot_record_save(const fb_geometry_t *);
#endif
