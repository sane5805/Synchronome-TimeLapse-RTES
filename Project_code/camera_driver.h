#ifndef _CAMERA_DRIVER_H_
#define _CAMERA_DRIVER_H_

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>
#include <mqueue.h>
#include <syslog.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <errno.h>
#include <string.h>
#include <signal.h>







////////////////////////////////////////////////////////////////////////
//                Global Defines
////////////////////////////////////////////////////////////////////////

#define START_UP_FRAMES         1
#define LAST_FRAMES             1
#define WRITEBACK_FRAMES        1810
#define FRAMES_TO_STORE         180
#define SIZEOF_RING             40



////////////////////////////////////////////////////////////////////////
//                Global Variables
////////////////////////////////////////////////////////////////////////

unsigned char yuyv_to_rgb_buffer[1280 * 960];
unsigned char rgb_to_negative_buffer[1280 * 960];

struct timespec thread_2_curr_time_val;
double thread2_start_time_val;

enum io_method 
{
	IO_METHOD_READ,
	IO_METHOD_MMAP,
	IO_METHOD_USERPTR,
};


void errno_exit(const char *s);


// for initialization of V4L2
void open_device(void);
void init_device(void);
void start_capturing(void);

int xioctl(int fh, int request, void *arg);
void errno_exit(const char *s);

void init_userp(unsigned int buffer_size);
void init_mmap(void);
void init_read(unsigned int buffer_size);

void stop_capturing(void);

// for un-initialization of V4L2
void uninit_device(void);
void close_device(void);


void mainloop(void);
void convert_yuyv_to_rgb(const void *p, int size);
void convert_rgb_to_negative(const void *p, int size);
void dump_image(const void *p, int size, unsigned int tag, struct timespec *time);


#endif // _CAMERA_DRIVER_H_
