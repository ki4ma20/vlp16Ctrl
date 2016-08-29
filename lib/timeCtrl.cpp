/*!
  \file
  \brief Function to control timer

  \author Kiyoshi MATSUO
  $Id$
*/

// include for maximum value of size_t
#include <limits>

#include "timeCtrl.h"

#if defined(APPLE_OS) || defined(LINUX_OS)
// include for gettimeofday
#include <time.h>
// include for gettimeofday
#include <sys/time.h>

// include for getrusage
#include <sys/resource.h>

// include for ulseep
#include <unistd.h>

#endif

enum{
  DAY_TIME_LENGTH = 255,
};

size_t GetNowTimeMicroSec(void)
{
    size_t time = 0;

#ifdef WINDOWS_OS
    SYSTEMTIME system_time;
    GetSystemTime(&system_time);

    time = system_time.wSecond * 1000000 + system_time.wMilliseconds * 1000;
#else
    struct timeval time_val;
    gettimeofday(&time_val, NULL);

    time = time_val.tv_sec * 1000000 + time_val.tv_usec;
#endif

    return time;
}

size_t GetNowTimeMicroSecRusage(void)
{
    size_t time = 0;

#ifdef WINDOWS_OS
    SYSTEMTIME system_time;
    GetSystemTime(&system_time);

    time = system_time.wSecond * 1000000 + system_time.wMilliseconds * 1000;
#else
    struct rusage resource;
    struct timeval time_val;
    getrusage(RUSAGE_SELF, &resource);

    time_val = resource.ru_stime;

    time = time_val.tv_sec * 1000000 + time_val.tv_usec;
#endif

    return time;
}

void sleep_milisecond(unsigned int wait_time_msec)
{
#ifdef WINDOWS_OS
    Sleep(wait_time_msec);
#else
    usleep(wait_time_msec * 1000);
#endif

    return;

}

TimeTheInterval::TimeTheInterval()
{

#ifdef WINDOWS_OS
    QueryPerformanceFrequency(&frequency);

    const double time_scale_milisecond = 1000.0;

    if (frequency.QuadPart != 0) {
        frequency_inverse = time_scale_milisecond / (double)frequency.QuadPart;
    } else {
        frequency_inverse = 0.0;
    }

    QueryPerformanceCounter(&past_count);
#else
    past_time = GetNowTimeMicroSec();
#endif

}

TimeTheInterval::~TimeTheInterval()
{
}

#ifdef WINDOWS_OS
void TimeTheInterval::RenewFrequency(void)
{
    QueryPerformanceFrequency(&frequency);

    const double time_scale_milisecond = 1000.0;

    if (frequency.QuadPart != 0) {
        frequency_inverse = time_scale_milisecond / (double)frequency.QuadPart;
    } else {
        frequency_inverse = 0.0;
    }

    return;
}

double TimeTheInterval::CalculateOneCountUsec(void)
{
    const double time_scale_usec = 1000.0;

    return frequency_inverse * time_scale_usec;

}

LARGE_INTEGER TimeTheInterval::GetPastCount(void) const
{
    return past_count;
}

void TimeTheInterval::SetPastCount(LARGE_INTEGER count)
{
    past_count = count;

    return;
}

#endif

void TimeTheInterval::SetIntervalStart(void)
{
#ifdef WINDOWS_OS
    QueryPerformanceFrequency(&frequency);

    const double time_scale_milisecond = 1000.0;

    if (frequency.QuadPart != 0) {
        frequency_inverse = time_scale_milisecond / (double)frequency.QuadPart;
    } else {
        frequency_inverse = 0.0;
    }

    QueryPerformanceCounter(&past_count);
#else
    past_time = GetNowTimeMicroSec();
#endif

  return;
}

size_t TimeTheInterval::TimeInterval(void) const
{
    size_t time = 0;

#ifdef WINDOWS_OS
    LARGE_INTEGER current_count;
    QueryPerformanceCounter(&current_count);

    LONGLONG count_difference = 0;

    if (current_count.QuadPart >= past_count.QuadPart) {
        count_difference = current_count.QuadPart - past_count.QuadPart;
    } else {
        const LONGLONG maximum = LLONG_MAX;
        count_difference = current_count.QuadPart + (maximum - past_count.QuadPart);
    }

    const double time_scale_usec = 1000.0;

    time = (size_t)((double)count_difference * frequency_inverse * time_scale_usec);

#else
    size_t now_time = GetNowTimeMicroSec();

    if (now_time >= past_time) {
        time = now_time - past_time;
    } else {
        const size_t maximum = std::numeric_limits<size_t>::max();
        time = now_time + (maximum - past_time);
    }

#endif

    return time;
}

size_t TimeTheInterval::TimeInterval(size_t start_time_usec) const
{
    size_t time = 0;

#ifdef WINDOWS_OS

    const double time_scale_usec_inverse = 0.001;
    double scaled_start_time = time_scale_usec_inverse * (double)start_time_usec;

    LARGE_INTEGER current_count;
    QueryPerformanceCounter(&current_count);

    LONGLONG count_difference = 0;

    if (current_count.QuadPart >= past_count.QuadPart) {
        count_difference = current_count.QuadPart - past_count.QuadPart;
    } else {
        const LONGLONG maximum = LLONG_MAX;
        count_difference = current_count.QuadPart + (maximum - past_count.QuadPart);
    }

    LARGE_INTEGER start_time_count;
    start_time_count.QuadPart =
        (LONGLONG)(scaled_start_time * (double)frequency.QuadPart);

    if (count_difference >= start_time_count.QuadPart) {
        count_difference = count_difference - start_time_count.QuadPart;
    } else {
        const LONGLONG maximum = LLONG_MAX;
        count_difference = count_difference + (maximum - start_time_count.QuadPart);
    }

    const double time_scale_usec = 1000.0;

    time = (size_t)((double)count_difference * frequency_inverse * time_scale_usec);

#else
    size_t now_time = GetNowTimeMicroSec();

    if (now_time >= past_time) {
        now_time = now_time - past_time;
    } else {
        const size_t maximum = std::numeric_limits<size_t>::max();
        now_time = now_time + (maximum - past_time);
    }

    if (now_time >= start_time_usec) {
        time = now_time - start_time_usec;
    } else {
        const size_t maximum = std::numeric_limits<size_t>::max();
        time = now_time + (maximum - start_time_usec);
    }


#endif

    return time;
}

size_t TimeTheInterval::GetNowTime(void) const
{
    size_t now_time = GetNowTimeMicroSec();

    return now_time;
}

size_t TimeTheInterval::GetPastTime(void) const
{
    return past_time;
}

void TimeTheInterval::SetPastTime(size_t time)
{
#ifdef WINDOWS_OS

    const double time_scale_usec_inverse = 0.001;
    double scaled_time = time_scale_usec_inverse * (double)time;

    past_count.QuadPart = (LONGLONG)(scaled_time * (double)frequency.QuadPart);

#else
    past_time = time;
#endif
    return;
}

void match_the_start_time(const TimeTheInterval &master,
                          TimeTheInterval &slave)
{

#ifdef WINDOWS_OS
    slave.SetPastCount(master.GetPastCount());
#else
    slave.SetPastTime(master.GetPastTime());
#endif

    return;
}

void make_day_string_of_local_time(char *day_string, char separator)
{
    time_t current_time;
    current_time = time(NULL);

    tm *time_pointer = NULL;
    time_pointer = localtime(&current_time);
    if (separator == 0) {
        strftime(day_string, DAY_STRING_LENGTH, "%Y%m%d", time_pointer);
    } else {
        strftime(day_string, DAY_STRING_LENGTH, "%Y %m %d", time_pointer);

        day_string[YEAR_STRING_LENGTH] = separator;
        day_string[YEAR_STRING_LENGTH + 1 + MONTH_STRING_LENGTH] = separator;

    }

    return;
}

void make_time_string_of_local_time(char *time_string, char separator)
{
    time_t current_time;
    current_time = time(NULL);

    tm *time_pointer = NULL;
    time_pointer = localtime(&current_time);
    if (separator == 0) {
        strftime(time_string, TIME_STRING_LENGTH, "%H%M%S", time_pointer);
    } else {
        strftime(time_string, TIME_STRING_LENGTH, "%H %M %S", time_pointer);

        time_string[HOUR_STRING_LENGTH] = separator;
        time_string[HOUR_STRING_LENGTH + 1 + MINUTE_STRING_LENGTH] = separator;

    }

    return;
}

void make_day_and_time_string_of_local_time(char *day_string, char day_separator,
                                            char *time_string, char time_separator)
{
    time_t current_time;
    current_time = time(NULL);

    tm *time_pointer = NULL;
    time_pointer = localtime(&current_time);

    if (day_separator == 0) {
        strftime(day_string, DAY_STRING_LENGTH, "%Y%m%d", time_pointer);
    } else {
        strftime(day_string, DAY_STRING_LENGTH, "%Y %m %d", time_pointer);

        day_string[YEAR_STRING_LENGTH] = day_separator;
        day_string[YEAR_STRING_LENGTH + 1 + MONTH_STRING_LENGTH] = day_separator;

    }

    if (time_separator == 0) {
        strftime(time_string, TIME_STRING_LENGTH, "%H%M%S", time_pointer);
    } else {
        strftime(time_string, TIME_STRING_LENGTH, "%H %M %S", time_pointer);

        time_string[HOUR_STRING_LENGTH] = time_separator;
        time_string[HOUR_STRING_LENGTH + 1 + MINUTE_STRING_LENGTH] = time_separator;

    }

    return;
}
