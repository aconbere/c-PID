#include "pid_time.h"

#if defined(ARDUINO)
#include <Arduino.h>
#elif defined(unix) || defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
#include <sys/time.h>
#endif
 
double getCurrentMilliseconds() {
#if defined(ARDUINO)
  return millis();
#elif defined(unix) || defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))
  static struct timeval t;
  gettimeofday(&t, NULL);
  return (double)(t.tv_usec / 1000);
#else
# error Unknown platform, cannot define getCurrentMilliseconds.
#endif
}
