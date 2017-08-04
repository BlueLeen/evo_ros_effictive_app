#include <string.h>
#include <sys/time.h>

#define EVODEBUG

#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1):__FILE__)

#ifdef EVODEBUG
#define evo_msg(fmt, args ...) \
{                           \
    struct timeval tm;  \
    gettimeofday(&tm, NULL);    \
    printf("[EVOMSG] [F:%s L:%d T:%ld.%ld]: "fmt, __FILENAME__, __LINE__,tm.tv_sec,tm.tv_usec,##args);  \
    printf("\n");       \
}
#else
#define evo_msg(fmt, args ...)
#endif

/* #undef ULTRASONICDEBUG */

#ifdef ULTRASONICDEBUG
#define UltrasonicMsg(fmt, args ...) \
{                           \
    struct timeval tm;  \
    gettimeofday(&tm, NULL);    \
    printf("[ULTRASONICMSG] [F:%s L:%d T:%ld.%ld]: "fmt, __FILENAME__, __LINE__,tm.tv_sec,tm.tv_usec,##args);  \
    printf("\n");       \
}
#else
#define UltrasonicMsg(fmt, args ...)
#endif


/* #undef MONITORDEBUG */

#ifdef MONITORDEBUG
#define MonitorMsg(fmt, args ...) \
{                           \
    struct timeval tm;  \
    gettimeofday(&tm, NULL);    \
    printf("[MONITORMSG] [F:%s L:%d T:%ld.%ld]: "fmt, __FILENAME__, __LINE__,tm.tv_sec,tm.tv_usec,##args);  \
    printf("\n");       \
}
#else
#define MonitorMsg(fmt, args ...)
#endif
