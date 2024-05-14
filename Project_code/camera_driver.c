
////////////////////////////////////////////////////////////////////////
//                Includes
////////////////////////////////////////////////////////////////////////

#include "camera_driver.h"
#include <stdbool.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include "time_spec.h"
#include <sys/time.h>


////////////////////////////////////////////////////////////////////////
//                Defines
////////////////////////////////////////////////////////////////////////

#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define HRES 640
#define VRES 480
#define HRES_STR	"640"
#define VRES_STR  	"480"
//#define HRES 320
//#define VRES 240
//#define HRES_STR "320"
//#define VRES_STR "240"

// Format is used by a number of functions, so made as a file global
//static struct v4l2_format fmt;

//enum io_method 
//{
//        IO_METHOD_READ,
//        IO_METHOD_MMAP,
//        IO_METHOD_USERPTR,
//};

struct buffer 
{
  void   *start;
  size_t  length;
};


#define FALSE               0
#define TRUE                1
#define NANOSEC_PER_SEC     1000000000.0
#define COUNT_NANO_SEC      1000000000.0



////////////////////////////////////////////////////////////////////////
//                Local Variables
////////////////////////////////////////////////////////////////////////

const int force_format = 1;
enum io_method   io = IO_METHOD_MMAP;
double fnow = 0.0, fstart = 0.0, fstop = 0.0, fnow_negative = 0.0;
struct timespec time_now, time_start, time_stop,time_now_negative;
struct timespec frame_time;
struct buffer *buffers;
unsigned int n_buffers;
int fd = -1;
struct v4l2_format fmt;
int framecnt = -START_UP_FRAMES;
int g_framesize;

char ppm_header[137] = "P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[21] = "frames/orig0000.ppm";



////////////////////////////////////////////////////////////////////////
//                Global Variables
////////////////////////////////////////////////////////////////////////
extern int thread_2_index;
extern int start_up_condition;
extern int transform_on_off;
extern unsigned char thread_2_temp_buffer[SIZEOF_RING][614400];
extern char ppm_uname_string[100];
extern char *dev_name;



////////////////////////////////////////////////////////////////////////
//                Local Functions
////////////////////////////////////////////////////////////////////////

int read_frame(void);

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


void yuv2rgb_float(float y, float u, float v, unsigned char *r, unsigned char *g, unsigned char *b);
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b);



////////////////////////////////////////////////////////////////////////
//                Function Definitions
////////////////////////////////////////////////////////////////////////

void convert_rgb_to_negative(const void *p, int size)
{
    int idx, new_idx;
    unsigned char *ptr = (unsigned char *)p;

	// original images -> no additional feature enabled
	if(TRUE == transform_on_off)
	{
		for(idx = 0, new_idx = 0; idx < size; idx = idx + 4, new_idx = new_idx + 6)
		{			
			// Negative transformation for the first set of pixels
			rgb_to_negative_buffer[new_idx] = 255 - ptr[new_idx];
			rgb_to_negative_buffer[new_idx + 1] = 255 - ptr[new_idx + 1];
			rgb_to_negative_buffer[new_idx + 2] = 255 - ptr[new_idx + 2];

			// Negative transformation for the second set of pixels
			rgb_to_negative_buffer[new_idx + 3] = 255 - ptr[new_idx + 3];
			rgb_to_negative_buffer[new_idx + 4] = 255 - ptr[new_idx + 4];
			rgb_to_negative_buffer[new_idx + 5] = 255 - ptr[new_idx + 5];
		}
	}
	// original + changed images -> additional feature enabled
	else if(FALSE == transform_on_off)
	{
		memcpy((void *)&(rgb_to_negative_buffer[0]), p, sizeof(rgb_to_negative_buffer));
	}
}


void mainloop(void)
{  
    fd_set fds;
    struct timeval tv;
    int r;

    if(!framecnt) 
    {
		clock_gettime(CLOCK_MONOTONIC, &time_start);
		fstart = (double)time_start.tv_sec + (double)time_start.tv_nsec / NANOSEC_PER_SEC;
    }

    FD_ZERO(&fds);
    FD_SET(fd, &fds);

    /* Timeout */
    tv.tv_sec = 5;
    tv.tv_usec = 0;

    r = select(fd + 1, &fds, NULL, NULL, &tv);
		
    if(-1 == r)
    {
		if (EINTR == errno)
			errno_exit("select");
    }

    if(0 == r)
    {
		fprintf(stderr, "select timeout\n");
		
		exit(EXIT_FAILURE);
    }

    
#if 1
    // Frame aquisition start time
    clock_gettime(CLOCK_MONOTONIC, &thread_2_curr_time_val); 
    thread2_start_time_val = (double)(thread_2_curr_time_val.tv_sec) + (((double)thread_2_curr_time_val.tv_nsec)/COUNT_NANO_SEC);
#endif

					 
    if(TRUE == read_frame())
    {
		if(framecnt > -1)
		{	
			clock_gettime(CLOCK_MONOTONIC, &time_now);
			
			fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / NANOSEC_PER_SEC;			
		}
    }
}


int read_frame()
{
    struct v4l2_buffer buf;
    unsigned int i;
	
	CLEAR(buf);
	
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if(-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
	{
		switch (errno)
		{
			case EAGAIN:
				return 0;
				
			case EIO:
				return 0;
				
			default:
				printf("mmap failure\n");
				errno_exit("VIDIOC_DQBUF");
		}
	}

	assert(buf.index < n_buffers);

	if(thread_2_index % 40 == 0)
		thread_2_index = 0;

	clock_gettime(CLOCK_REALTIME, &frame_time);
	memcpy((void *)&thread_2_temp_buffer[thread_2_index][0], buffers[buf.index].start, buf.bytesused);
		
	if(-1 == xioctl(fd, VIDIOC_QBUF, &buf)) {
		errno_exit("VIDIOC_QBUF");
	}	
    
	return 1;
}


void open_device(void)
{
    struct stat st;

    if(-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n",
				dev_name, errno, strerror(errno));
		
		exit(EXIT_FAILURE);
    }

    if(!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev_name);
		
		exit(EXIT_FAILURE);
    }

    fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if(-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
				dev_name, errno, strerror(errno));
		
		exit(EXIT_FAILURE);
    }
}


void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    if(-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
		if (EINVAL == errno) 
		{
			fprintf(stderr, "%s is no V4L2 device\n", dev_name);
			exit(EXIT_FAILURE);
		}
		else
		{
			errno_exit("VIDIOC_QUERYCAP");
		}
    }

    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
		fprintf(stderr, "%s is no video capture device\n",dev_name);
		
		exit(EXIT_FAILURE);
    }

	if(!(cap.capabilities & V4L2_CAP_STREAMING))
	{
		fprintf(stderr, "%s does not support streaming i/o\n",
				dev_name);
			
		exit(EXIT_FAILURE);
	}

    /* Select video input, video standard and tune here. */
    
    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if(-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
		{
			switch (errno)
			{
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
    }
    else
    {
		/* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(force_format)
    {
		syslog(LOG_INFO, "FORCING FORMAT\n");
		fmt.fmt.pix.width       = HRES;
		fmt.fmt.pix.height      = VRES;

		// Specify the Pixel Coding Format here

		// This one works for Logitech C200
		fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
					
		//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
		//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

		// Would be nice if camera supported
		//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
		//fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

		//fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
		fmt.fmt.pix.field       = V4L2_FIELD_NONE;

		if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
			errno_exit("VIDIOC_S_FMT");

		/* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
		printf("ASSUMING FORMAT\n");
		/* Preserve original settings as set by v4l2-ctl for example */
		if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
			errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
    
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;


	init_mmap();
}


int xioctl(int fh, int request, void *arg)
{
    int r;

    do 
    {
		r = ioctl(fh, request, arg);

    } while (-1 == r && EINTR == errno);

    return r;
}


void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}


void init_read(unsigned int buffer_size)
{
    buffers = calloc(1, sizeof(*buffers));
    
    if(!buffers) 
    {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
    }

    buffers[0].length = buffer_size;
    buffers[0].start = malloc(buffer_size);

    if(!buffers[0].start) 
    {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
    }
}


void init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 6;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if(-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
    {
		if (EINVAL == errno) 
		{
			fprintf(stderr, "%s does not support "
					"memory mapping\n", dev_name);
			
			exit(EXIT_FAILURE);
		}
		else 
		{
			errno_exit("VIDIOC_REQBUFS");
		}
    }

    if(req.count < 2) 
    {
		fprintf(stderr, "Insufficient buffer memory on %s\n", 
				dev_name);
		
		exit(EXIT_FAILURE);
    }

    buffers = calloc(req.count, sizeof(*buffers));

    if(!buffers) 
    {
		fprintf(stderr, "Out of memory\n");
		
		exit(EXIT_FAILURE);
    }

    for(n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index  = n_buffers;

		if(-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
						errno_exit("VIDIOC_QUERYBUF");

		buffers[n_buffers].length = buf.length;
		
		buffers[n_buffers].start = mmap(NULL /* start anywhere */,
										buf.length,
										PROT_READ | PROT_WRITE /* required */,
										MAP_SHARED /* recommended */,
										fd,
										buf.m.offset);

		if(MAP_FAILED == buffers[n_buffers].start)
			errno_exit("mmap");
    }
}


void init_userp(unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
	{
		if (EINVAL == errno)
		{
			fprintf(stderr, "%s does not support "
					 "user pointer i/o\n", dev_name);
			exit(EXIT_FAILURE);
		}
		else
		{
			errno_exit("VIDIOC_REQBUFS");
		}
    }

    buffers = calloc(4, sizeof(*buffers));

    if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		
		exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers)
    {
		buffers[n_buffers].length = buffer_size;
		buffers[n_buffers].start = malloc(buffer_size);

		if(!buffers[n_buffers].start)
		{
			fprintf(stderr, "Out of memory\n");
			
			exit(EXIT_FAILURE);
		}
    }
}


void start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

	for(i = 0; i < n_buffers; ++i) 
	{
		syslog(LOG_INFO, "allocated buffer %d\n", i);
		struct v4l2_buffer buf;

		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if(-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		errno_exit("VIDIOC_QBUF");
	}
	
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	
	if(-1 == xioctl(fd, VIDIOC_STREAMON, &type))
		errno_exit("VIDIOC_STREAMON");
}


void stop_capturing(void)
{
    enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
		errno_exit("VIDIOC_STREAMOFF");
    
    if(-1 < framecnt) 
	{
		clock_gettime(CLOCK_MONOTONIC, &time_stop);
		fstop = (double)time_stop.tv_sec + (double)time_stop.tv_nsec / NANOSEC_PER_SEC;
    }
}


void uninit_device(void)
{
    unsigned int i;

	for(i = 0; i < n_buffers; ++i)
		if (-1 == munmap(buffers[i].start, buffers[i].length))
			errno_exit("munmap");

    free(buffers);
}


void close_device(void)
{
    if(-1 == close(fd))
		errno_exit("close");

    fd = -1;
}


void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
    int r1, g1, b1;

    // replaces floating point coefficients
    int c = y-16, d = u - 128, e = v - 128;       

    // Conversion that avoids floating point
    r1 = (298 * c           + 409 * e + 128) >> 8;
    g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
    b1 = (298 * c + 516 * d           + 128) >> 8;

    // Computed values may need clipping.
    if (r1 > 255) r1 = 255;
    if (g1 > 255) g1 = 255;
    if (b1 > 255) b1 = 255;

    if (r1 < 0) r1 = 0;
    if (g1 < 0) g1 = 0;
    if (b1 < 0) b1 = 0;

    *r = r1 ;
    *g = g1 ;
    *b = b1 ;
}


void convert_yuyv_to_rgb(const void *p, int size)
{
    int i, newi, y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;
	
	g_framesize = size;
	
	// Pixels are YU and YV alternating, so YUYV which is 4 bytes
	// We want RGB, so RGBRGB which is 6 bytes
	for(i = 0, newi = 0; i < size; i = i + 4, newi = newi + 6)
	{
		y_temp = (int)pptr[i]; 
		u_temp = (int)pptr[i + 1]; 
		y2_temp = (int)pptr[i + 2]; 
		v_temp = (int)pptr[i + 3];     
		
		yuv2rgb(y_temp, u_temp, v_temp, &yuyv_to_rgb_buffer[newi], &yuyv_to_rgb_buffer[newi + 1], &yuyv_to_rgb_buffer[newi + 2]);
		yuv2rgb(y2_temp, u_temp, v_temp, &yuyv_to_rgb_buffer[newi+3], &yuyv_to_rgb_buffer[newi + 4], &yuyv_to_rgb_buffer[newi + 5]);
	}

	if(framecnt > -1)   
		g_framesize = size;
}


void dump_image(const void *p, int size, unsigned int tag, struct timespec *time)
{
	int written, total, dumpfd;

	snprintf(&ppm_dumpname[11], 9, "%04d", tag);
	strncat(&ppm_dumpname[15], ".ppm", 5);
	dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

	snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
	strcat(&ppm_header[14], " sec ");
	snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
	strcat(&ppm_header[29], " msec \n");
	strcat(&ppm_header[36], ppm_uname_string);
	strcat(&ppm_header[122], ""HRES_STR" "VRES_STR"\n255\n");
	written = write(dumpfd, ppm_header, sizeof(ppm_header)-1);

	total = 0;

	do
	{
		written=write(dumpfd, p, size);
		total+=written;
	} while(total < size);

	clock_gettime(CLOCK_MONOTONIC, &time_now);
	fnow = (double)time_now.tv_sec + (double)time_now.tv_nsec / NANOSEC_PER_SEC;

	syslog(LOG_INFO, "Frame_written_to_flash %d at %lf %d bytes\n",framecnt, (fnow-fstart), total);

	close(dumpfd);
}

