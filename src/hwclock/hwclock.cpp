#include "hwclock.h"

void setHWClock(const char *hostname) {
    // NTP Sync
    struct tm tm;
    time_t t;
    struct timeval tv;
    configTime(0, 0, hostname);

    // Get time from NTP
    getLocalTime(&tm);

    // Set time on HW clock
    t = mktime(&tm);
    tv = { .tv_sec = t };
    settimeofday(&tv, NULL);
}
