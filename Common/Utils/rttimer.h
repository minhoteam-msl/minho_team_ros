/* This header defines methods for creating a timed thread
*/

#ifndef rttimer_h
#define rttimer_h

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/timerfd.h>

/// \brief struct that controls the timing with the
/// file descriptor timerfd
typedef struct periodic_info
{
	int timer_fd;
	unsigned long long wakeups_missed;
   unsigned long int period_us;
   int id;

}periodic_info;

/// \brief function to initialize properly a periodic_info struct
/// creating a timerfd file descriptor and initializing the timer
/// \param period - period in microseconds
/// \param info - pointer to periodic_info struct to be filled
static inline int make_periodic (unsigned int period, periodic_info *info)
{
	int ret;
	unsigned int ns;
	unsigned int sec;
	int fd;
	struct itimerspec itval;

	/* Create the timer */
	fd = timerfd_create (CLOCK_REALTIME, 0);
	info->wakeups_missed = 0;
	info->timer_fd = fd;
	if (fd == -1)
		return fd;

	/* Make the timer periodic */
	sec = period/1000000;
	ns = (period - (sec * 1000000)) * 1000;
	itval.it_interval.tv_sec = sec;
	itval.it_interval.tv_nsec = ns;
	itval.it_value.tv_sec = sec;
	itval.it_value.tv_nsec = ns;
	ret = timerfd_settime (fd, 0, &itval, NULL);
	return ret;
}

/// \brief wait function to wait for timing. Whenever it reaches
/// the specified timing, it allows the file descriptor to be read
///  \param info - pointer to periodic_info struct that contains 
// the file descriptor
static inline void wait_period (periodic_info *info)
{
	unsigned long long missed;
	int ret;

	/* Wait for the next timer event. If we have missed any the
	   number is written to "missed" */
	ret = read (info->timer_fd, &missed, sizeof (missed));
	if (ret == -1){
		perror ("read timer");
		return;
	}

	/* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
	if (missed > 0)
		info->wakeups_missed += (missed - 1);
}

#endif
