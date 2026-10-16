#ifndef DUAL_REALTIME_H
#define DUAL_REALTIME_H
/* Startup copies .RamFunc through the existing .data load image. Keep the
 * acquisition/current/protection path independent of flash cache activity
 * from foreground geometry calculations. Native host tests use normal text. */
#if defined(__arm__) || defined(__thumb__)
#define DUAL_REALTIME __attribute__((section(".RamFunc")))
#else
#define DUAL_REALTIME
#endif
#endif
