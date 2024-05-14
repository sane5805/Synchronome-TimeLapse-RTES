
#define _GNU_SOURCE


////////////////////////////////////////////////////////////////////////
//                Includes
////////////////////////////////////////////////////////////////////////
#include "main.h"

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
#include <string.h>
#include <signal.h>

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


#include "camera_driver.h"
#include "time_spec.h"
#include "services.h"



////////////////////////////////////////////////////////////////////////
//                Defines
////////////////////////////////////////////////////////////////////////

#define FALSE                           0
#define TRUE                            1
#define TOTAL_CORES_COUNT               4
#define TOTAL_THREADS_COUNT             5
#define IMAGE_STORAGE_THREAD_CORE       3

#define FRAMES_COUNT_180                1
#define FRAMES_COUNT_1800               2

#define ADDITIIONAL_FEATURE_DISABLE     1
#define ADDITIIONAL_FEATURE_ENABLE      2

// have to do something!
#define UNAME_PATH_LENGTH               100

// Message queue data - start
#define TOTAL_MQ_LEN	                300
#define MQ_NAME_THREAD_2	            "/Thread_2"
#define MQ_NAME_THREAD_3	            "/Thread_3"
#define MQ_NAME_THREAD_4                "/Thread_4"
#define MQ_NAME_THREAD_5                "/Thread_5"
// Message queue data - end


#define TEN_MS                   		10 * 1000000 // in nsec -> 10ms
#define NANO_S                  		1000000000.0


#define TEST_1_HZ                       1
#define TEST_10_HZ                      1


////////////////////////////////////////////////////////////////////////
//                        Local Variables
////////////////////////////////////////////////////////////////////////

// for freq selection 1Hz or 10Hz
int freq_option;

// for frames count selection 180 or 1800
int frames_count_option;
int number_of_frames_to_store;

// for additional feature option RGB or -ve
int additional_feature_option;
int additional_feature_on_off;


// for selecting camera device
char *camera;

unsigned char thread_2_temp_buffer[40][614400];


/* Message queue data - start */
struct mq_attr attr_thread_2;
struct mq_attr attr_thread_3;
struct mq_attr attr_thread_4;
struct mq_attr attr_thread_5;

mqd_t mqdes_thread_2;
mqd_t mqdes_thread_3;
mqd_t mqdes_thread_4;
mqd_t mqdes_thread_5;
/* Message queue data - end */


/* thread locks data - start */
sem_t thread_1_lock; 
sem_t thread_2_lock; 
sem_t thread_3_lock;
sem_t thread_4_lock;
sem_t thread_5_lock;
sem_t thread_6_lock;

int thread_1_abort;
int thread_2_abort;
int thread_3_abort;
int thread_4_abort;
int thread_5_abort;
/* thread locks data - end */

/* Sequencer timer data - start */
timer_t sequencer_timer;
struct itimerspec timer = {{1,0}, {1,0}};
struct itimerspec prev_itime;
/* Sequencer timer data - end */


int thread_2_index;


unsigned char thread_3_temp_buffer[614400];

typedef struct {
  int threadIdx;
} threadParams_t;


/* array for core selecting - start */
// { Thread_1, Thread_2, Thread_3, Thread_4, Thread_5 }
int core_selection_1Hz[] = {1, 1, 1, 2, 2};
int core_selection_10Hz[] = {1, 1, 2, 2, 2};
/* array for core selecting - end */


char ppm_uname_string[UNAME_PATH_LENGTH];

// keeps track of the frames 
int curr_frame_count;

// for triggring exit exit of all services
int stop_testing = FALSE;


// Calculate size of frame pointer
size_t frame_size = sizeof(void *); 
    
// Calculate the size of the frame count
size_t frame_count_size = sizeof(int);
// Calculate the offset where frame count should be copied
size_t frame_count_offset = sizeof(void *);        
    
// Calculate the size of the timestamp
size_t timestamp_size = sizeof(struct timespec);
// Calculate the offset where the timestamp should be copied
size_t timestamp_offset = sizeof(void *) + sizeof(int);

size_t frame_len = sizeof(void *) + sizeof(int) + sizeof(struct timespec); 


////////////////////////////////////////////////////////////////////////
//                		Global Variables
////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////
//                		Global Functions
////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////
//                		Local Functions
////////////////////////////////////////////////////////////////////////

void initialize_variables();
void initialize_semaphores();
void initialize_timer();


// Message Queue setup and release
void create_message_queue();
void close_message_queue();

void unlock_semaphores(void);
void close_services(void);

void timer_callback(int id);
// semaphore - stop

// Thread functions
void *thread_1(void *threadp);
void *thread_2(void *threadp);
void *thread_3(void *threadp);
void *thread_4(void *threadp);
void *thread_5(void *threadp);
void *thread_image_dumping(void *threadp);


double cal_time(struct timespec *time);



////////////////////////////////////////////////////////////////////////
//                Function Definitions
////////////////////////////////////////////////////////////////////////

void display_colored_menu() 
{
    printf("\033[1;36m"); // Set text color to cyan
    printf("===================================================================\n");
    printf("      Welcome to Synchronome Demo              \n");
    printf("===================================================================\n\n\n");
    printf("\033[0m"); // Reset text color to default
    
    
    printf("\033[1;36m"); // Set text color to cyan
    printf("===============================================\n");
    printf("      Select Frequency                          \n");
    printf("===============================================\n");
    printf("\033[0m"); // Reset text color to default
    
    printf("\033[1;33m"); // Set text color to yellow
    printf("1. 1  Hz \n");
    printf("2. 10 Hz \n");
    printf("\033[1;36m"); // Set text color to cyan
    printf("===============================================\n");
    printf("\033[0m"); // Reset text color to default
    
    freq_option = (int)(getchar() - '0');
    getchar();  // Consume the newline character
    
    printf("\033[1;36m"); // Set text color to cyan
    printf("===============================================\n\n\n");
    printf("\033[0m"); // Reset text color to default
    
    
    
    
    printf("\033[1;36m"); // Set text color to cyan
    printf("===============================================\n");
    printf("      Additional feature enable/disable ?      \n");
    printf("===============================================\n");
    printf("\033[0m"); // Reset text color to default
    
    printf("\033[1;33m"); // Set text color to yellow
    printf("1. disable \n");
    printf("2. enable \n");
    printf("\033[1;36m"); // Set text color to cyan
    printf("===============================================\n");
    printf("\033[0m"); // Reset text color to default
    
    additional_feature_option = (int)(getchar() - '0');
    getchar();  // Consume the newline character
    
    printf("\033[1;36m"); // Set text color to cyan
    printf("===============================================\n\n\n");
    printf("\033[0m"); // Reset text color to default
    
    
        
    
    printf("\033[1;36m"); // Set text color to cyan
    printf("===============================================\n");
    printf("      Select Frames count                      \n");
    printf("===============================================\n");
    printf("\033[0m"); // Reset text color to default
    
    printf("\033[1;33m"); // Set text color to yellow
    printf("1. 180  frames \n");
    printf("2. 1800 frames \n");
    printf("\033[1;36m"); // Set text color to cyan
    printf("===============================================\n");
    printf("\033[0m"); // Reset text color to default
    
    frames_count_option = (int)(getchar() - '0');
    getchar();  // Consume the newline character
    
    printf("\033[1;36m"); // Set text color to cyan
    printf("===============================================\n\n");
    printf("\033[0m"); // Reset text color to default
    
    printf("\033[1;36m"); // Set text color to cyan
    printf("===================================================================\n\n\n");
    printf("\033[0m"); // Reset text color to default
  



    // User wants 180 frames
    if(frames_count_option == FRAMES_COUNT_180)
        number_of_frames_to_store = 180;
	
	// User wants 1800 frames
    if(frames_count_option == FRAMES_COUNT_1800)
        number_of_frames_to_store = 1800;
    
	// User wants result images as -ve
    if(additional_feature_option == ADDITIIONAL_FEATURE_ENABLE)
        additional_feature_on_off = 1;
    
	// User wants result images in normal RGB format
    if(additional_feature_option == ADDITIIONAL_FEATURE_DISABLE)
        additional_feature_on_off = 0;
}

void initialize_timer()
{
    // timer to trigger Sequencer function in every 10ms

    // timer creation
    timer_create(CLOCK_REALTIME, NULL, &sequencer_timer);
    
    signal(SIGALRM, (void(*)()) timer_callback);
    
    // 10 ms timer initialization
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_nsec = TEN_MS;  // 10 ms
    
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_nsec = TEN_MS;     // 10 ms
    
    timer_settime(sequencer_timer, 0, &timer, &prev_itime);
}

void open_services()
{
    // shutdown all services
    thread_1_abort = FALSE; 
    thread_2_abort = FALSE; 
    thread_3_abort = FALSE; 
    thread_4_abort = FALSE; 
    thread_5_abort = FALSE; 
}

void initialize_variables()
{
    open_services();
    
    thread_2_index = 0;

    
    additional_feature_on_off = 1;    

    
    FILE *fp;
    
	char path[UNAME_PATH_LENGTH];
    
    strcpy(path, "");
	
    strcpy(ppm_uname_string, "");
    
    fp = popen("uname -a", "r");
    
	if (fp == NULL)
        /* Handle error */;
    
    while(fgets(path, UNAME_PATH_LENGTH, fp) != NULL)
        printf("%s", path);
    
    strcpy(ppm_uname_string, "#");
    
	strncat(ppm_uname_string, path, strlen(path)); 
  
    pclose(fp);
}

void initialize_semaphores()
{
    sem_init(&thread_1_lock, 0, 0);
    sem_init(&thread_2_lock, 0, 0);
    sem_init(&thread_3_lock, 0, 0);
    sem_init(&thread_4_lock, 0, 0);
    sem_init(&thread_5_lock, 0, 0);
}

void* (*func_ptr_arr[])() = {
                              thread_1, 
                              thread_2, 
                              thread_3,
                              thread_4,
                              thread_5
                            };

void service_thread_create( int thread_num, 
                            int rt_max_prio, 
                            pthread_attr_t *rt_sched_attr, 
                            struct sched_param *rt_param, 
                            threadParams_t *threadParams,
                            pthread_t *threads)
{  
    rt_param[thread_num].sched_priority = rt_max_prio - (thread_num + 1);
    
    pthread_attr_setschedparam(&rt_sched_attr[thread_num], &rt_param[thread_num]);
    
    int ret = pthread_create(&threads[thread_num], 
							&rt_sched_attr[thread_num], 
							(*func_ptr_arr[thread_num]), 
							(void *)&(threadParams[thread_num]));
    
    if(ret < 0)
        printf("Thread creation failed, error = (%d)\n", thread_num);

    
    pthread_detach(threads[thread_num]);
}

void create_message_queue()
{
    attr_thread_2.mq_maxmsg = TOTAL_MQ_LEN;
    attr_thread_3.mq_maxmsg = TOTAL_MQ_LEN;
    attr_thread_4.mq_maxmsg = TOTAL_MQ_LEN;
    attr_thread_5.mq_maxmsg = TOTAL_MQ_LEN;
    
	//                        frame's address + frame num   + timestamp    
    attr_thread_2.mq_msgsize = sizeof(void *) + sizeof(int) + sizeof(struct timespec);
    attr_thread_3.mq_msgsize = sizeof(void *) + sizeof(int) + sizeof(struct timespec);
    attr_thread_4.mq_msgsize = sizeof(void *) + sizeof(int) + sizeof(struct timespec);
    attr_thread_5.mq_msgsize = sizeof(void *) + sizeof(int) + sizeof(struct timespec);
    
    attr_thread_2.mq_flags = 0;
    attr_thread_3.mq_flags = 0;
    attr_thread_4.mq_flags = 0;
    attr_thread_5.mq_flags = 0;
    
    mqdes_thread_2 = mq_open(MQ_NAME_THREAD_2 , O_CREAT | O_RDWR, S_IRWXU, &attr_thread_2);
    mqdes_thread_3 = mq_open(MQ_NAME_THREAD_3 , O_CREAT | O_RDWR, S_IRWXU, &attr_thread_3);
    mqdes_thread_4 = mq_open(MQ_NAME_THREAD_4 , O_CREAT | O_RDWR, S_IRWXU, &attr_thread_4);
    mqdes_thread_5 = mq_open(MQ_NAME_THREAD_5 , O_CREAT | O_RDWR, S_IRWXU, &attr_thread_5);
}

void close_message_queue()
{
    mq_close(mqdes_thread_2);
    mq_close(mqdes_thread_3);
    mq_close(mqdes_thread_4);
    mq_close(mqdes_thread_5);
    
    mq_unlink(MQ_NAME_THREAD_2);   
    mq_unlink(MQ_NAME_THREAD_3);
    mq_unlink(MQ_NAME_THREAD_4);
    mq_unlink(MQ_NAME_THREAD_5);
}

double cal_time(struct timespec *time)
{
	double ret = (double)(time->tv_sec) + (((double)time->tv_nsec)/NANO_S);
    
	return ret;
}

void unlock_semaphores()
{
    // unlock semaphores of all the threads
    sem_post(&thread_1_lock);
    sem_post(&thread_2_lock); 
    sem_post(&thread_3_lock); 
    sem_post(&thread_4_lock); 
    sem_post(&thread_5_lock), 
    sem_post(&thread_6_lock);  
}

void close_services()
{
    // shutdown all services
    thread_1_abort = TRUE; 
    thread_2_abort = TRUE; 
    thread_3_abort = TRUE; 
    thread_4_abort = TRUE; 
    thread_5_abort = TRUE; 
}

void disable_timer()
{
    // disable timer
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_nsec = 0;
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_nsec = 0;
    
    timer_settime(sequencer_timer, 0, &timer, &prev_itime);   
}

void timer_callback(int id)
{
    // unlocking for Sequencer (thread_1) 
    sem_post(&thread_1_lock);
    
    // executes when all the requested frames are captured
    if(stop_testing) 
	{    
        disable_timer();
        close_services();
        unlock_semaphores();
    }
}



// Thread 1 - Thread Sequencer
// This service is called at 100 Hz frequency => 10 ms for both 1Hz & 10Hz case
// CORE - 1
void *thread_1(void *threadp)
{ 
	double thread1_start_time;
    double curr_time;
    struct timespec curr_time_val;
    unsigned long long thread_1_ran_count = 0;

    printf("\nInside thread 1 function\n");

    while(1)
    {
        // wait till the semaphore to get released by timer callback function
        sem_wait(&thread_1_lock);
        
        // Code comes here after every 10 ms! (100 Hz) => 10ms
	
	#if 1
        // Thread 1 start time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        thread1_start_time = cal_time(&curr_time_val);
    #endif
    

        /********************  Sequencer start *******************/
        // Executes when all the frames are successfully dumped
        if(thread_1_abort) break;
        
        // Counter for Thread no.  
        thread_1_ran_count++;
        
        // 1 Hz
        if(freq_option == 1) 
        {
            // below threads will run at 10*10ms => 100ms   (10 Hz)
            if((thread_1_ran_count % 10) == 0) 
            {
                sem_post(&thread_2_lock);
                sem_post(&thread_3_lock);
            }

            // below thread will run at 10*100ms => 1000ms -> 1 sec  (1 Hz)
            if((thread_1_ran_count % 100) == 0)
                sem_post(&thread_4_lock);

            // below thread will run at 10*100ms => 1000ms -> 1 sec  (1 Hz)
            if((thread_1_ran_count % 100) == 0)
                sem_post(&thread_5_lock); 
       
            // run dumping thread after every 500 ms
            if((thread_1_ran_count % 50) == 0) 
                sem_post(&thread_6_lock);
        }

		// 10 Hz
        else if(freq_option == 2) 
        {
            // below thread will run at 5*10ms => 50ms   (20 Hz)
            if((thread_1_ran_count % 5) == 0) 
                sem_post(&thread_2_lock);

            // below thread will run at 5*10ms => 50ms   (20 Hz)
            if((thread_1_ran_count % 5) == 0) // SN test 50ms 
                sem_post(&thread_3_lock); 

            // below thread will run at 10*10ms => 100ms   (10 Hz)
            if((thread_1_ran_count % 10) == 0)
                sem_post(&thread_4_lock);

            // below thread will run at 10*10ms => 100ms   (10 Hz)
            if((thread_1_ran_count % 10) == 0)
                sem_post(&thread_5_lock); 
       
            // run dumping thread after every 50 ms
            if((thread_1_ran_count % 5) == 0) 
                sem_post(&thread_6_lock);
        }
        /********************  sequencer end *******************/


	#if 1
        // gather current time for calculating execution time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        curr_time = cal_time(&curr_time_val);
	#endif
	
        syslog(LOG_INFO, "[RTES_FINAL_PROJECT_SYNCHRONOME]__Thread_1 thread_1_ran_count %llu & Execution_time(sec) = %6.9lf\n", 
                         thread_1_ran_count, 
                         curr_time - thread1_start_time);
    }
    
	printf("\nExiting thread 1 function\n");
    pthread_exit((void *)0);
}


// Thread 2 - Image Capture / Frame Acquisition
//        thread freq
// 1Hz    => 10 Hz capture
// 10 Hz  => 20 Hz capture
// CORE - 1
void *thread_2(void *threadp)
{
	double thread2_start_time;
    double curr_time;
    struct timespec curr_time_val;
    unsigned long long thread_2_ran_count = 0; 
    int frame_count_temp = 0;
    void * thread_1_temp_ptr;
	                
    // frame for sending via message queue                
    // Buffer element -> image + frame no. + timestamp
    char frame[frame_len];
	
	printf("\nInside thread 2 function\n");

    while(1)
    {
        // wait till the semaphore to get released at 10Hz frequency
        sem_wait(&thread_2_lock);
        
        
        /********************  Frame capture start *******************/
        // Executes when all the frames are successfully dumped
        if(thread_2_abort) break;
        
        // Counter for Thread no. 
        thread_2_ran_count++;

	
        // reponsible for aquisition
        mainloop();
        
        // dynamic memeory allocated for frames
        thread_1_temp_ptr = (void *) malloc((614400 * sizeof(unsigned char)));  
        if(!thread_1_temp_ptr)
            printf("Error while thread_1_temp_ptr mallocing");
        
      
        // image data copying to memory pointed by ptr
        memcpy(thread_1_temp_ptr, &thread_2_temp_buffer[thread_2_index], (614400 * sizeof(unsigned char)));
	
        // Image address now pasted in buffer's first 4bytes        
        memcpy(frame, &thread_1_temp_ptr, sizeof(void *));
        
        // increment index for next frame
        thread_2_index++;
        
        // get the live frame count
        frame_count_temp = curr_frame_count;
        
        
        
        // Copy frame count to the buffer
        memcpy(&(frame[frame_count_offset]), &frame_count_temp, frame_count_size);

        // Copy timestamp to the buffer
        memcpy(&(frame[timestamp_offset]), &frame_time, timestamp_size);
      

	#if 1
        // gather current time for calculating execution time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        curr_time = cal_time(&curr_time_val);
	#endif
      
        // send the buffer to thread_3  
        mq_send(mqdes_thread_2, frame, frame_len, 30);
        /********************  Frame capture end *******************/
      

		
		// Syslog the execution time
		syslog(LOG_INFO, "[RTES_FINAL_PROJECT_SYNCHRONOME]__Thread_2 thread_2_ran_count %llu & Execution_time(sec) = %6.9lf\n", 
						 thread_2_ran_count, 
						 curr_time - thread2_start_time_val);
    }

    printf("\nExiting thread 2 function\n");
    pthread_exit((void *)0);
}


// Thread 3 - Selects some frames from the captured frames to process
//        thread freq
// 1Hz    => 10 Hz
// 10 Hz  => 20 Hz
// CORE - 1 for 1Hz & CORE - 2 for 10Hz 
void *thread_3(void *threadp)
{
	double thread3_start_time;
    double curr_time;
    struct timespec curr_time_val;
    unsigned long long thread_3_ran_count = 0;
	int temp_frame_num = 0;
	
	void *temp_buffer;

    // frame for sending via message queue                 
    // Buffer element -> image + frame no. + timestamp
    char frame[frame_len];
	
	

    int frame_skip_util = 0;
	
	// global frame count initialized once here
	curr_frame_count = 0; 
   
	
    // open the queue for recieving data sent by image capture thread   
    mqdes_thread_2 = mq_open(MQ_NAME_THREAD_2, O_CREAT | O_RDWR, S_IRWXU, &attr_thread_2);
	
	
	// 1 Hz 
	if(freq_option == 1)
		curr_frame_count = -50; // 5s delay
	
	// 10 Hz	
	else if(freq_option == 2)
		curr_frame_count = -200; // 10s delay


	printf("\nInside thread 3 function\n");	
	
    while(1)
    {
        // wait till the semaphore to get released
        sem_wait(&thread_3_lock);
        
        
        // Thread's start time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        thread3_start_time = cal_time(&curr_time_val);
        
        
        /********************  frame selection start *******************/
        // Executes when all the frames are successfully dumped
        if(thread_3_abort) break;
        
        // Thread_3 ran count for syslog  
        thread_3_ran_count++;
        
        
        char msg_ptr[frame_len];
        unsigned int msg_prio;
        int frame_num;
        struct timespec frame_time;
        void *buffer_ptr;     


        // receive data sent from thread 2
        mq_receive(mqdes_thread_2, msg_ptr, (size_t)frame_len, &msg_prio);


        // frame's data extraction
        memcpy(&buffer_ptr, msg_ptr, frame_count_offset);
        
		// extract frame number
		memcpy(&frame_num, &(msg_ptr[frame_count_offset]), frame_count_size);
        
		// extract timestamp
		memcpy(&frame_time, &(msg_ptr[timestamp_offset]), timestamp_size);
        
        // copy full buffer into temp buffer
        memcpy(thread_3_temp_buffer, buffer_ptr, sizeof(thread_3_temp_buffer));
        
        
        
        free(buffer_ptr); buffer_ptr = NULL;
          
        
        if(curr_frame_count >= 0)
        {            
            // we have to work on evey 10th frame which means
            // 1 frame forwarded after every 1sec 
            
			// 1 Hz handing
			// chooses every 10th frame
            if(freq_option == 1 && frame_skip_util % 10 == 0)
            {
				temp_buffer = (void *) malloc(sizeof(thread_3_temp_buffer)); 
				if(!temp_buffer)
					printf("temp_buffer malloc failed - %lld\n", thread_3_ran_count);


				// retrival of recieved frame from thread_2 into temp buffer
				// current frame's copy is now in heap  
				memcpy(temp_buffer, thread_3_temp_buffer, sizeof(thread_3_temp_buffer));
				
				// Copy actual frame to buffer
				memcpy(frame, &temp_buffer, frame_size);
				  
				// Copy frame count to the buffer       
				memcpy(&(frame[frame_count_offset]), &temp_frame_num, frame_count_size);
				
				// Copy timestamp to the buffer
				memcpy(&(frame[timestamp_offset]), &frame_time, timestamp_size);
				
				// send the buffer for thread_4 
				mq_send(mqdes_thread_3, frame, frame_len, 30);

				
				// frame no. gets updated after evey 1s
				// this is sent in message queue
				temp_frame_num++;
            }
			
			// 10 Hz handing
			// chooses every 2nd frame
            if(freq_option == 2 && frame_skip_util % 2 == 0)
            { 
				//mq send 
				temp_buffer = (void *) malloc(sizeof(thread_3_temp_buffer)); 
				if(!temp_buffer)
					printf("temp_buffer malloc failed - %lld\n", thread_3_ran_count);


				// retrival of recieved frame from thread_2 into temp buffer
				// current frame's copy is now in heap 
				memcpy(temp_buffer, thread_3_temp_buffer, sizeof(thread_3_temp_buffer));
				
				// copying buffer's address onto frame (in first 4 bytes)
				memcpy(frame, &temp_buffer, frame_size);
				  
				// Copy frame count to the buffer
				memcpy(&(frame[frame_count_offset]), &temp_frame_num, frame_count_size);
				
				// Copy timestamp to the buffer
				memcpy(&(frame[timestamp_offset]), &frame_time, timestamp_size);
				
				// send the buffer for thread_4 
				mq_send(mqdes_thread_3, frame, frame_len, 30);

				
				// frame no. gets updated after evey 1s
				// this is sent in message queue
				temp_frame_num++;
            }
                    
            frame_skip_util++;
        }
        
		// keeps the count of frames thread 3 is receiving from thread 2
        curr_frame_count++;
        
        // gather current time for calculating execution time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        curr_time = cal_time(&curr_time_val);
        
		
        // Syslog the execution time           
        syslog(LOG_INFO, "[RTES_FINAL_PROJECT_SYNCHRONOME]__Thread_3 thread_3_ran_count %llu & Execution_time(sec) = %6.9lf\n", 
                         thread_3_ran_count, 
                         curr_time - thread3_start_time);
    }
    
    printf("\nExiting thread 3 function\n");
    pthread_exit((void *)0);
}


// Thread 4 - Converts YUYV into RGB
//        thread freq
// 1Hz    => 1 Hz
// 10 Hz  => 10 Hz
// CORE - 2 
void *thread_4(void *threadp)
{ 
	double thread4_start_time;
    double curr_time;
    struct timespec curr_time_val;
    unsigned long long thread_4_ran_count = 0;

    void *temp_buffer;
		
    // frame for sending via message queue                
    // Buffer element -> image + frame no. + timestamp
    char frame[frame_len];


    // open the queue for recieving data sent by frame selection thread   
    mqdes_thread_3 = mq_open(MQ_NAME_THREAD_3, O_CREAT | O_RDWR, S_IRWXU, &attr_thread_3);


	printf("\nInside thread 4 function\n");

    while(1)
    {
        // wait till the semaphore to get released
        sem_wait(&thread_4_lock);
        
		
        /********************  YUYV to RGB start *******************/
        // Executes when all the frames are successfully dumped
        if(thread_4_abort) break;
        
        // Thread_4 ran count for syslog
        thread_4_ran_count++;
        
                
        // getting frame no., frame's timestamp and frame is saved in yuyv_to_rgb_buffer    
        char msg_ptr[frame_len];
        unsigned int msg_prio;
        int frame_num;
        struct timespec frame_time;
        void *buffer_ptr; 
        	
        // receive data sent from thread 3
        mq_receive(mqdes_thread_3, msg_ptr, (size_t)frame_len, &msg_prio);

        
        // frame's data extraction
        memcpy(&buffer_ptr, msg_ptr, frame_size);
        
		// extract frame number
		memcpy(&frame_num, &(msg_ptr[frame_count_offset]), frame_count_size);
        
		// extract timestamp
		memcpy(&frame_time, &(msg_ptr[timestamp_offset]), timestamp_size);
        
		
	#if 1
		// Thread's start time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        thread4_start_time = cal_time(&curr_time_val);
	#endif


        
        // converts YUYV into RGB
        convert_yuyv_to_rgb(buffer_ptr, 614400);


        
        free(buffer_ptr); buffer_ptr = NULL;


        temp_buffer = (void *)malloc(sizeof(yuyv_to_rgb_buffer)); 
        if(temp_buffer == NULL)
            printf("temp_buffer malloc failed - %lld\n", thread_4_ran_count);

        
        // retrival of recieved frame from thread_3 into temp buffer
        // current frame's copy is now in heap 
        memcpy(temp_buffer, yuyv_to_rgb_buffer, sizeof(yuyv_to_rgb_buffer));
        
        // Copy actual frame to buffer
        memcpy(frame, &temp_buffer, frame_size);
        
        // Copy frame count to the buffer 
        memcpy(&(frame[frame_count_offset]), &frame_num, frame_count_size);
        
        // Copy timestamp to the buffer 
        memcpy(&(frame[timestamp_offset]), &frame_time, timestamp_size);
        
		
    #if 1
		// gather current time for calculating execution time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        curr_time = cal_time(&curr_time_val);
    #endif 
        
		
        // send the buffer for thread_5 
        mq_send(mqdes_thread_4, frame, frame_len, 30);


        syslog(LOG_INFO, "[RTES_FINAL_PROJECT_SYNCHRONOME]__Thread_4 thread_4_ran_count %llu & Execution_time(sec) = %6.9lf\n", 
                         thread_4_ran_count, 
                         curr_time - thread4_start_time);
    }
    
    printf("\nExiting thread 4 function\n");
    pthread_exit((void *)0);
}


// Thread 5 - Converts RGB into -ve if selected
//        thread freq
// 1Hz    => 1 Hz
// 10 Hz  => 10 Hz
// CORE - 2 
void *thread_5(void *threadp)
{   
	double thread5_start_time;
    double curr_time;
    struct timespec curr_time_val;
    unsigned long long thread_5_ran_count = 0;
    
	void *temp_buffer;

    // frame for sending via message queue                 
    // Buffer element -> image + frame no. + timestamp    
    char frame[frame_len];


    // open the queue for recieving data sent by thread_4  
    mqdes_thread_4 = mq_open(MQ_NAME_THREAD_4, O_CREAT | O_RDWR, S_IRWXU, &attr_thread_4);

	
	printf("\nInside thread 5 function\n");
	
    while(1)
    {
        // wait till the semaphore to get released
        sem_wait(&thread_5_lock);
        
        
        // Executes when all the frames are successfully dumped
        if(thread_5_abort) break;
        
        // Thread_5 ran count for syslog
        thread_5_ran_count++;
        

        // Converts the image to -ve if specified by user 
        char msg_ptr[frame_len];
        unsigned int msg_prio;
        int frame_num;
        struct timespec frame_time;
        void *buffer_ptr; 
		
        // receive data sent from thread 4
        mq_receive(mqdes_thread_4, msg_ptr, (size_t)frame_len, &msg_prio);


	#if 1
        // Thread's start time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        thread5_start_time = cal_time(&curr_time_val);
	#endif 
        
        // frame's data extraction
        memcpy(&buffer_ptr, msg_ptr, frame_size);
        
		// frame's data extraction
		memcpy(&frame_num, &(msg_ptr[frame_count_offset]), frame_count_size);
        
		// extract frame number
		memcpy(&frame_time, &(msg_ptr[timestamp_offset]), timestamp_size);


               
        // Converts the image to -ve if specified by user 
        convert_rgb_to_negative(buffer_ptr, 614400);


         
        free(buffer_ptr); buffer_ptr = NULL;


        temp_buffer = (void *)malloc(sizeof(rgb_to_negative_buffer));
        if(!temp_buffer)
            printf("malloc failed inside thread 5");
        
		
        // retrival of recieved frame from thread_4 into temp buffer
        // current frame's copy is now in heap         
        memcpy(temp_buffer, rgb_to_negative_buffer, sizeof(rgb_to_negative_buffer));
        
		// Copy actual frame to buffer
        memcpy(frame, &temp_buffer, frame_size);
        
		// Copy frame count to the buffer
        memcpy(&(frame[frame_count_offset]), &frame_num, frame_count_size);
        
		// Copy timestamp to the buffer
        memcpy(&(frame[timestamp_offset]), &frame_time, timestamp_size);
    

	#if 1
        // gather current time for calculating execution time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        curr_time = cal_time(&curr_time_val);
	#endif

    
        // send the buffer for thread_6 for dumping image (saving) 
        mq_send(mqdes_thread_5, frame, frame_len, 30);
        
		
        syslog(LOG_INFO, "[RTES_FINAL_PROJECT_SYNCHRONOME]__Thread_5 thread_5_ran_count %llu & Execution_time(sec) = %6.9lf\n", 
                         thread_5_ran_count, 
                         curr_time - thread5_start_time);
    }
    
    printf("\nExiting thread 5 function\n");
    pthread_exit((void *)0);
}


// Thread 6 - Image Storage(saving)
//        thread freq
// 1Hz    => 1 Hz
// 10 Hz  => 10 Hz
// CORE - 3 
void *thread_image_dumping(void *threadp)
{   
    double thread6_start_time;
	double curr_time;
	struct timespec curr_time_val;
	unsigned long long image_dumping_thread_count = 0;
	size_t frame_len = frame_size + frame_count_size + timestamp_size;
	
    int frames_stored = 0;

    char msg_ptr[frame_len];
    unsigned int msg_prio;
    int frame_num = 0;
    struct timespec frame_time;
    void *buffer_ptr;
    

    // open the queue for recieving data sent by thread_5  
    mqdes_thread_5 = mq_open(MQ_NAME_THREAD_5 , O_CREAT | O_RDWR, S_IRWXU, &attr_thread_5);
  
  
	printf("\nInside image dump thread function\n");
	
    while(1)
    {
        // Executes when all the frames are successfully dumped
        if(frame_num >= number_of_frames_to_store) 
        {   
            // Will diable everything in next timer callback
            stop_testing = TRUE;
            break;
        }
    
	#if 1
        // Thread's start time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        thread6_start_time = cal_time(&curr_time_val);
	#endif
    
        // Thread_6 ran count for syslog
        image_dumping_thread_count++;
       
	   
	   
        // receive data sent from thread 5
        mq_receive(mqdes_thread_5, msg_ptr, (size_t)frame_len, &msg_prio);
       
	   
     
        // frame's data extraction
        memcpy(&buffer_ptr, msg_ptr, frame_size);
        
		// extract frame number
        memcpy(&frame_num, &(msg_ptr[frame_count_offset]), frame_count_size);
        
		// extract timestamp
        memcpy(&frame_time, &(msg_ptr[timestamp_offset]), timestamp_size);
        
        
        dump_image(buffer_ptr, ((614400 * 6)/4), frame_num, &frame_time);

        
        free(buffer_ptr); buffer_ptr = NULL;

	
	#ifdef 1
        // on order of up to milliseconds of latency to get time
        clock_gettime(CLOCK_MONOTONIC, &curr_time_val); 
        curr_time = cal_time(&curr_time_val);
    #endif
	
        
        syslog(LOG_INFO, "[RTES_FINAL_PROJECT_SYNCHRONOME]__Thread_6 thread_6_ran_count %llu & Execution_time(sec) = %6.9lf\n", 
                         image_dumping_thread_count, 
                         curr_time - thread6_start_time);
    }
    
	printf("\nExiting image dump thread function\n");
    pthread_exit((void *)0); 
}


int main(int argc, char *argv[])
{
    struct timespec current_time_val, current_time_res;
    int i;
    struct timespec start_time_val;

    cpu_set_t cpu_set;
        
    pthread_t threads[TOTAL_THREADS_COUNT];
    threadParams_t threadParams[TOTAL_THREADS_COUNT];
    pthread_attr_t rt_sched_attr[TOTAL_THREADS_COUNT];

    int rt_max_prio;
    
    struct sched_param rt_param[TOTAL_THREADS_COUNT];
    struct sched_param main_param;


    pid_t mainpid;

	// select camera device
    camera = "/dev/video0";
    
    // initialize variables
    initialize_variables();
	
	// launch the menu
    display_colored_menu(); 
    
	// create message queue
	create_message_queue();
    
    
    printf("Synchronome Project code Start\n");
    
    
    // initialization of V4L2
    open_device();
    printf("Camera Opened\n");
    
    init_device();
    printf("Camera Initialization Successful\n");
    
    start_capturing();
    printf("Camera start capturing\n");


	// Clears all the CPU cores in the set
	// This initializes the set to an empty set
    CPU_ZERO(&cpu_set);
	
	// This effectively populates the set with all available CPU cores
    for(i = 0; i < TOTAL_CORES_COUNT; i++) 
	{
        CPU_SET(i, &cpu_set);
    }


    initialize_semaphores();
    
    
    mainpid = getpid();

    rt_max_prio = sched_get_priority_max(SCHED_FIFO);

    int response = sched_getparam(mainpid, &main_param);
    
	main_param.sched_priority = rt_max_prio;
    
    response = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
    if(response < 0) {
        perror("set scheduler error");
    }
    
	
	
	int cpuidx;
	cpu_set_t curr_cpu;
	
	// for thread 1, 2, 3, 4, 5
    for(i = 0; i < TOTAL_THREADS_COUNT; i++)
    {
        CPU_ZERO(&curr_cpu);
        
        if(freq_option == 1) {
            cpuidx = core_selection_1Hz[i];
        }
        else if(freq_option == 2) {
            cpuidx = core_selection_10Hz[i];
		}
		
        CPU_SET(cpuidx, &curr_cpu);

        pthread_attr_init(&rt_sched_attr[i]);
        pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
        pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &curr_cpu);

        threadParams[i].threadIdx=i;
    }
    
    // for image dumping thread(6)
    pthread_t writeback_thread;
    threadParams_t writeback_threadparam;
    pthread_attr_t writeback_sched_attr;
    
	CPU_ZERO(&curr_cpu);
    cpuidx = IMAGE_STORAGE_THREAD_CORE;	// core 3
	CPU_SET(cpuidx, &curr_cpu);
    pthread_attr_init(&writeback_sched_attr);
	pthread_attr_setaffinity_np(&writeback_sched_attr, sizeof(cpu_set_t), &curr_cpu);
    
    
	
	
    // Sequencer service 
    service_thread_create(0, rt_max_prio, rt_sched_attr, rt_param, threadParams, threads);
    printf("Thread 1 - Sequencer Created\n");
    
    // Frame Acquisition service
    service_thread_create(1, rt_max_prio, rt_sched_attr, rt_param, threadParams, threads);
    printf("Thread 2 - Image Capture Created\n");
    
    // Frame selection service
    service_thread_create(2, rt_max_prio, rt_sched_attr, rt_param, threadParams, threads);  
    printf("Thread 3 - Image Capture Created\n");
    
	// YUYV to RGB conversion service
    service_thread_create(3, rt_max_prio, rt_sched_attr, rt_param, threadParams, threads); 
    printf("Thread 4 YuYV to RGB Created\n");
    
	// RGB to -ve conversion service
    service_thread_create(4, rt_max_prio, rt_sched_attr, rt_param, threadParams, threads); 
    printf("Thread 5 RGB to -ve if selected Created\n");
    
    writeback_threadparam.threadIdx = 9;
    // Image dumping service
	pthread_create(&writeback_thread, &writeback_sched_attr, thread_image_dumping, (void *)&writeback_threadparam);
    printf("Thread 6 Image dumping thread created\n");
        
        
    initialize_timer();
    printf("Sequencer timing interval set\n");
    
    
    if((response = pthread_join(writeback_thread, NULL) < 0)) {
        perror("Join failed");
    }
  
  
    // Camera shutdown
    stop_capturing();
    uninit_device();
    close_device();
    printf("Camera Closed\n");

    close_message_queue();
    
    printf("Synchronome Project code End\n");
    
    return 0;
}

