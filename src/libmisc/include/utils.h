#ifndef _UTILS_H_INCLUDED
#define _UTILS_H_INCLUDED

#include <stdint.h>
#include <time.h>

extern int verbose;

void dump(const char *title, uint8_t *buf, int len);
uint8_t crc8_data(const uint8_t *data, uint8_t len);

#define MAYBE_UNUSED __attribute__((unused))
#define PACKED __attribute__((packed))

#define _unlikely(cond) __builtin_expect((cond), 0)
#define _likely(cond) __builtin_expect((cond), 1)

#if defined(cplusplus) || defined(__cplusplus)
extern "C" {
#endif

inline uint64_t get_timestamp()
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

#if defined(cplusplus) || defined(__cplusplus)
}
#endif

#endif  // _UTILS_H_INCLUDED
