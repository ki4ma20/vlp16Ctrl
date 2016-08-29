#ifndef TIME_CTRL_H
#define TIME_CTRL_H
/*!
  \file
  \brief Functions to control timer

  \author Kiyoshi MATSUO
  $Id$
*/

#include "environmentCtrl.h"

#include <stddef.h>

/*!
  \brief function to get now time micro sec
  \attention this function returns millisecond time on Windows OS
*/
extern size_t GetNowTimeMicroSec(void);

/*!
  \brief function to get now time micro sec
  \attention this function returns millisecond time on Windows OS
  \attention I don't know right usage.
 */
extern size_t GetNowTimeMicroSecRusage(void);

/*!
  \brief function to sleep [milisecond]
  \attnetion this function uses Sleep on Windows OS
  \attention this function uses usleep (wait_time_msec * 1000) on Other OS
*/
extern void sleep_milisecond(unsigned int wait_time_msec);

/*!
  \brief class to time the interval
*/
class TimeTheInterval{
private:
    //! interval start time
    size_t past_time;

#ifdef WINDOWS_OS
    LARGE_INTEGER frequency;
    double frequency_inverse;
    LARGE_INTEGER past_count;
#endif

public:
    TimeTheInterval();
    ~TimeTheInterval();

#ifdef WINDOWS_OS
    /*!
      \brief method to renew frequency
      \attention this function set frequency_inverse is 0.0 if frequency is zero
      \attention this function is implemented for only Windows OS
    */
    void RenewFrequency(void);

    /*!
      \brief method to calculate one count time [usec]
      \attention this function return 0.0 if frequency_inverse is 0.0
      \attention this function is implemented for only Windows OS
    */
    double CalculateOneCountUsec(void);

    /*!
      \brief method to get past count
    */
    LARGE_INTEGER GetPastCount(void) const;

    /*!
      \brief method to set past count
    */
    void SetPastCount(LARGE_INTEGER count);

#endif
    /*!
      \brief method to set interval start time
     */
    void SetIntervalStart(void);

    /*!
      \brief method to time the interval
     */
    size_t TimeInterval(void) const;

    /*!
      \brief method to calculate interval
      \todo Check on windows
    */
    size_t TimeInterval(size_t start_time_usec) const;

    /*!
      \brief get now time
     */
    size_t GetNowTime(void) const;

    /*!
      \brief get past count
    */
    size_t GetPastTime(void) const;

    /*!
      \brief set past count
      \todo Check on windows
    */
    void SetPastTime(size_t time);

};

/*!
  \brief function to match the start time
*/
extern void match_the_start_time(const TimeTheInterval &master,
                                 TimeTheInterval &slave);

//! constants for day-time string
enum DAY_TIME_STRING_CONSTANT {

    //! length of year
    YEAR_STRING_LENGTH = 4,

    //! length of month
    MONTH_STRING_LENGTH = 2,

    //! length of day string 4 + 1 + 2 + 1 + 2 + 1
    DAY_STRING_LENGTH = 11,

    //! length of hour
    HOUR_STRING_LENGTH = 2,

    //! length of minute
    MINUTE_STRING_LENGTH = 2,

    //! length of time string 2 + 1 + 2 + 1 + 2 + 1
    TIME_STRING_LENGTH = 9,
};

/*!
  \brief function to make day string of local time
  \attention this function does not check length of the buffer
  \attention if separator is 0, not set separator
*/
extern void make_day_string_of_local_time(char *day_string, char separator);

/*!
  \brief function to make time string of local time
  \attention this function does not check length of the buffer
  \attention if separator is 0, not set separator
*/
extern void make_time_string_of_local_time(char *time_string, char separator);

/*!
  \brief function to make time string of local time
  \attention this function does not check length of the buffer
  \attention if separator is 0, not set separator
*/
extern void make_day_and_time_string_of_local_time(char *day_string, char day_separator,
                                                   char *time_string, char time_separator);

#endif // TIME_CTRL_H
