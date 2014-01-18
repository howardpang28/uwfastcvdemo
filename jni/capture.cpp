/*
 *  External USB webcam demo through V4L2 interface.
 *
 *  This file manages the USB webcam and ensures a clean interface to
 *  any program that interfaces to it. It manages the init, run and cleanup
 *  of all the resources to the webcam.
 *
 *  This program is property of University of Waterloo and can be used and 
 *  distributed without restrictions.
 *
 *  File: capture.cpp
 *  Author: Howard Pang
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <asm/types.h>          /* for videodev2.h */

#include <linux/videodev2.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>

#include <fastcv/fastcv.h>
#include <pthread.h>
#include <time.h>

#include "capture.h"
#include "render.h"

#define LOG_TAG    "USBFastDemoJNI"
#define DEV_NAME   "/dev/video0"
#define SET_WIDTH   320
#define SET_HEIGHT  240

#define CLEAR(x) memset (&(x), 0, sizeof (x))

unsigned int width;
unsigned int height;

static char *outbuf = NULL;
static char dev_name[20];
static int fd = -1;
static int s_frames;

// these buffers are used for the driver
static unsigned int n_buffers = 0;
struct buffer *buffers = NULL;

// these buffers are used for the fastcv rgba
static unsigned int w_fbuf = 0;
static unsigned int r_fbuf = 1;
static unsigned int n_fbuffers = 2;
static struct buffer *fbuffers = NULL;
static struct buffer *ybuffers = NULL;
static void* tempBuffer = NULL;

// The mutex is needed for the circular buffer used in grab frame
pthread_mutex_t mutexGrabFrame;

//------------------------------------------------------------------------------
//  This function is for error handling. Any errors that calls this function
//  is a fatal error and will abort the application.
//------------------------------------------------------------------------------
static void errno_exit(const char *s)
{
	printf("%s error %d, %s\n", s, errno, strerror(errno));
	exit (EXIT_FAILURE);
}

//------------------------------------------------------------------------------
//  wrapper for ioctl call with busy wait error handling
//------------------------------------------------------------------------------
static int xioctl(int fd, int request, void *arg)
{
	int r;

	do r = ioctl (fd, request, arg);
	while (-1 == r && EINTR == errno);

	return r;
}

//------------------------------------------------------------------------------
//  This function manages the grabbing of the frame from the USB webcam. When a 
//  frame is needed, call this function. A memcpy is invoked. 
//  The size is the size of the frame in bytes, and p must be fcvMemAlloc'd.
//------------------------------------------------------------------------------
int grabFrame(char* p, int size)
{
  	pthread_mutex_lock( &mutexGrabFrame );
    if (fbuffers[r_fbuf].start) {
        processFrame( (char*)(ybuffers[r_fbuf].start), (char*)(fbuffers[r_fbuf].start), (int)width, (int)height, 4);       
        memcpy(p, fbuffers[r_fbuf].start, size);

  	    pthread_mutex_unlock( &mutexGrabFrame );
        return 0;
    } else {
  	    pthread_mutex_unlock( &mutexGrabFrame );
        return 1;
    }
}

//------------------------------------------------------------------------------
//  This function is the grayscale version of grabFrame()
//------------------------------------------------------------------------------
int grabYFrame(char* p, int size)
{
  	pthread_mutex_lock( &mutexGrabFrame );
    if (ybuffers[r_fbuf].start) {
        memcpy(p, ybuffers[r_fbuf].start, size);

  	    pthread_mutex_unlock( &mutexGrabFrame );
        return 0;
    } else {
  	    pthread_mutex_unlock( &mutexGrabFrame );
        return 1;
    }
}

//------------------------------------------------------------------------------
// core code to process per frame
// p is a pointer to the frame and the size is
// width*height*bpp (bytes per pixel)
// RGBA = 4bpp, YUY2 = 2bpp
//------------------------------------------------------------------------------
static void process_image(const void *p)
{
    char* uv = (char*) calloc( width*height, sizeof(char));    

    for (unsigned int i = 0; i < width*height*2; i++) {
        if (i%2 == 0)
            ((char*)(ybuffers[w_fbuf].start))[i/2] = ((char*)p)[i];
        else
            uv[i/2] = ((char*)p)[i];    // interleaved u and v
    }

    // Colour space conversion
    // input:  YUV 4:2:2 YUY2(YUYV) ordering
    // output: RGBA interleaved, 8888
    fcvColorYCbCr422PseudoPlanarToRGBA8888u8((uint8_t*)ybuffers[w_fbuf].start, (uint8_t*)uv, width, height, 0, 0, (uint8_t*)fbuffers[w_fbuf].start, 0);

    // Circular buffer manipulation
  	pthread_mutex_lock( &mutexGrabFrame );    
    w_fbuf ^= 1;
    r_fbuf ^= 1;
  	pthread_mutex_unlock( &mutexGrabFrame );    

    free(uv);
}

static int read_frame(void)
{
    struct v4l2_buffer buf;
	unsigned int i;

	CLEAR (buf);
	buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	// Exchanges a buffer with the driver from kernel by dequeueing the driver
	if(xioctl (fd, VIDIOC_DQBUF, &buf) == -1) {
		switch (errno) {
		case EAGAIN:
			return 0;
		case EIO:
			/* Could ignore EIO, see spec. */
			/* fall through */
		default:
			errno_exit ("VIDIOC_DQBUF");
		}
	}

	// Terminates the program if buf.index is greater than possible buffers
	assert(buf.index < n_buffers);

	// Sotres the size of each frame
	s_frames = buf.length;
	process_image (buffers[buf.index].start);

	// Once the buf is processed, enqueue the used buf
	if(xioctl (fd, VIDIOC_QBUF, &buf) == -1)
		errno_exit ("VIDIOC_QBUF");

	return 1;
}

void cameraMain(void)
{
    // set to use HW units, but doesn't work
    // fcvSetOperationMode( (fcvOperationMode)FASTCV_OP_PERFORMANCE );

    // create the fastcv buffers
    // fbuffer for RGB
    // ybuffer for Y
	fbuffers = (buffer*) calloc(n_fbuffers, sizeof(*buffers));
    ybuffers = (buffer*) calloc(n_fbuffers, sizeof(*buffers));
    for (unsigned int i = 0; i < n_fbuffers; i++) {
        fbuffers[i].length = height*width*4;
        fbuffers[i].start  = fcvMemAlloc( height*width*4, 16);

        ybuffers[i].length = height*width;
        ybuffers[i].start  = fcvMemAlloc( height*width, 16);
    }

    // this is used for the filter and scaling of the greyscale image
    tempBuffer  = fcvMemAlloc( height*width, 16);

    while(1) {
		for (;;) {
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(fd, &fds);

			/* Timeout. */
			tv.tv_sec = 2;
			tv.tv_usec = 0;

			r = select(fd + 1, &fds, NULL, NULL, &tv);

			if (-1 == r) {
				if (EINTR == errno)
					continue;
				errno_exit("select");
			}

			if (0 == r) {
				printf("select timeout\n");
				exit(EXIT_FAILURE);
			}

			if (read_frame())
				break;

			/* EAGAIN - continue select loop. */
		}
	}

    // free the fastcv buffers
    for (unsigned int i = 0; i < n_fbuffers; i++) {
        fcvMemFree( fbuffers[i].start);
        fcvMemFree( ybuffers[i].start);
    }
    fcvMemFree( tempBuffer);
}

static void stop_capturing(void)
{
    enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
	    errno_exit ("VIDIOC_STREAMOFF");
	
	LOGI(LOG_TAG, "%s: stopped capturing from device\n", __func__);
}

static void start_capturing(void)
{
	unsigned int i;
	enum v4l2_buf_type type;

	// enqueues the as many empty buffers as the driver has before the stream starts
	for (i = 0; i < n_buffers; ++i) {
		struct v4l2_buffer buf;

		CLEAR (buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = i;

		// Exchanges a buffer with the driver from kernel by enqueueing the driver
		if (xioctl (fd, VIDIOC_QBUF, &buf) == -1)
			errno_exit ("VIDIOC_QBUF");
	}

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	// starts streaming from the driver
	if (xioctl (fd, VIDIOC_STREAMON, &type) == -1)
		errno_exit ("VIDIOC_STREAMON");

	LOGI(LOG_TAG, "%s: starting to capture from device\n", __func__);
}

static void uninit_device(void)
{
	unsigned int i;

    for (i = 0; i < n_buffers; ++i)
	    if (munmap(buffers[i].start, buffers[i].length) == -1)
		    errno_exit ("munmap");

	free(buffers);
	
	LOGI(LOG_TAG, "%s: sucessfully un-initialized device\n", __func__);
}

static void init_mmap(void)
{
	struct v4l2_requestbuffers req;

	CLEAR (req);

	req.count	= 4;
	req.type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory	= V4L2_MEMORY_MMAP;

	// Checks if MMAP is supported
	if (xioctl (fd, VIDIOC_REQBUFS, &req) == -1) {
		if (EINVAL == errno) {
			printf("%s does not support "
					 "memory mapping\n", dev_name);
			exit (EXIT_FAILURE);
		} else {
			errno_exit ("VIDIOC_REQBUFS");
		}
    }

	// Checks if sufficient buffer memory exists
	if (req.count < 2) {
		printf("Insufficient buffer memory on %s\n",
				 dev_name);
		exit (EXIT_FAILURE);
	}

	// Allocates memory
	buffers = (buffer*) calloc(req.count, sizeof(*buffers));
	if (!buffers) {
		printf("Out of memory\n");
		exit (EXIT_FAILURE);
	}

	// Buffer management
	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR (buf);
		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;

		// queries/gets the status of buffers in kernel
		if (xioctl (fd, VIDIOC_QUERYBUF, &buf) == -1)
				errno_exit ("VIDIOC_QUERYBUF");

		// copies it over to buffers[] in userspace
		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start =
				mmap (NULL /* start anywhere */,
				buf.length,
				PROT_READ | PROT_WRITE /* required */,
				MAP_SHARED /* recommended */,
				fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start)
				errno_exit ("mmap");
	}
	
	LOGI(LOG_TAG, "%s: sucessfully initialized mmap\n", __func__);
}

static void init_device(void)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;	// video cropping and scaling abilities
	struct v4l2_crop crop;			// get/set cropping rectangle
	struct v4l2_format fmt;
	unsigned int min;

	// Note that all V4L2 ioctl/xioctl calls are in the format of:
	// int ioctl(int fd, int request, struct v4l2_capability *argp);

	// Checks if the device driver is a V4L2 device
	if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
		if (EINVAL == errno) {
			printf("%s is no V4L2 device\n", dev_name);
			exit (EXIT_FAILURE);
		} else {
			errno_exit ("VIDIOC_QUERYCAP");
		}
	}

	// Checks if there are video capture capabilities
	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		printf("%s is no video capture device\n", dev_name);
		exit (EXIT_FAILURE);
	}

	// Checks for R/W or streaming capabilities
		if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
			printf("%s does not support streaming i/o\n", dev_name);
			exit (EXIT_FAILURE);
		}

	/* Select video input, video standard and tune here. */
	CLEAR(cropcap);
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (xioctl(fd, VIDIOC_CROPCAP, &cropcap) == 0) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (xioctl (fd, VIDIOC_S_CROP, &crop) == -1) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {	
		/* Errors ignored. */
	}

	CLEAR(fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	// VIDIOC_G_FMT gets current format to find out what is supported
	// this is done as a good practice
	if (xioctl (fd, VIDIOC_G_FMT, &fmt) == -1) {
		errno_exit("VIDIOC_G_FMT");
	}
	CLEAR (fmt);

#if 1
	// These parameters detail the output formats!!
	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = SET_WIDTH; 
	fmt.fmt.pix.height      = SET_HEIGHT;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field       = V4L2_FIELD_NONE;

	// VIDIOC_S_FMT sets the format
	if (xioctl (fd, VIDIOC_S_FMT, &fmt) == -1)
		errno_exit ("VIDIOC_S_FMT");
#endif

    // The actual set height and width
    width = fmt.fmt.pix.width;
    height = fmt.fmt.pix.height;

    /* Note VIDIOC_S_FMT may change width and height. */
	LOGI(LOG_TAG, "%s: Set size image %i\n", __func__, fmt.fmt.pix.sizeimage);
	LOGI(LOG_TAG, "%s: Set bytesperline %i\n", __func__, fmt.fmt.pix.bytesperline);
	LOGI(LOG_TAG, "%s: Set width %i height %i\n", __func__, fmt.fmt.pix.width, fmt.fmt.pix.height);
	LOGI(LOG_TAG, "%s: Set colour space %i\n", __func__, fmt.fmt.pix.colorspace);
	
	// Allocate the output buffer
	outbuf = (char*) calloc (fmt.fmt.pix.sizeimage * 3, sizeof(*outbuf));
	
    init_mmap();
}

static void close_device(void)
{
	if (close (fd) == -1)
		errno_exit("close");
	fd = -1;
	
	LOGI(LOG_TAG, "%s: sucessfully closed %s\n", __func__, dev_name);
}

static void open_device(void)
{
	// The struct stat gives information about the file,
	// in this case, the driver since linux is a file-based OS
	struct stat st; 

	// Checks if it exists
	if (stat(dev_name, &st) == -1) {
        LOGI(LOG_TAG, "Cannot identify '%s': %d, %s\n",
				dev_name, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}

	// Checks if it is a character device driver
	if (!S_ISCHR(st.st_mode)) {
        LOGI(LOG_TAG, "%s is no device\n", dev_name);
		exit (EXIT_FAILURE);
	}

	// Open the device driver
	fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

	// Check if sucessfully opened
	if (fd == -1) {
   		LOGI(LOG_TAG, "Cannot open '%s': %d, %s\n",
				 dev_name, errno, strerror (errno));
		exit (EXIT_FAILURE);
	}
    LOGI(LOG_TAG, "%s: sucessfully opened %s\n", __func__, dev_name);    
}

void initCamera(int *w, int* h) {
   char sVersion[32];  
   
   fcvSetOperationMode( (fcvOperationMode) FASTCV_OP_PERFORMANCE );

   fcvGetVersion(sVersion, 32);
   LOGI(LOG_TAG, "Using FastCV version %s \n", sVersion );

   LOGI(LOG_TAG, "Starting JNI FastCV app");
	strcpy (dev_name, DEV_NAME);

	open_device ();
	init_device ();
	start_capturing ();
    LOGI(LOG_TAG, "Successful init device"); 
    *w = width;
    *h = height;

    return;
}

void closeCamera() {
    stop_capturing ();
	uninit_device ();
	close_device ();
    LOGI(LOG_TAG, "Successfully closed device!");
    return;
}


//==============================================================================
// JNI Function Declarations
//==============================================================================

//------------------------------------------------------------------------------
// Called upon initialization of the camera
//------------------------------------------------------------------------------
JNIEXPORT void JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_initCamera (JNIEnv * env, jobject obj)
{
    int w, h;
    initCamera(&w, &h);
}

//------------------------------------------------------------------------------
// Called upon destruction of the camera
//------------------------------------------------------------------------------
JNIEXPORT void JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_closeCamera (JNIEnv * env, jobject obj)
{
    closeCamera();
}

//------------------------------------------------------------------------------
// Called by the main function of the Camera Thread
//------------------------------------------------------------------------------
JNIEXPORT void JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_cameraMain (JNIEnv * env, jobject obj)
{
	cameraMain();
}
