#ifndef _CAPTURE_
#define _CAPTURE_

/*
 *  External USB webcam demo through V4L2 interface.
 *
 *  This file definition manages the USB webcam and ensures a clean interface to
 *  any program that interfaces to it. It manages the init, run and cleanup
 *  of all the resources to the webcam.
 *
 *  This program is property of University of Waterloo and can be used and 
 *  distributed without restrictions.
 *
 *  File: capture.h
 *  Author: Howard Pang
 */

#define LOGV(TAG,...) __android_log_print(ANDROID_LOG_VERBOSE, TAG,__VA_ARGS__)
#define LOGD(TAG,...) __android_log_print(ANDROID_LOG_DEBUG  , TAG,__VA_ARGS__)
#define LOGI(TAG,...) __android_log_print(ANDROID_LOG_INFO   , TAG,__VA_ARGS__)
#define LOGW(TAG,...) __android_log_print(ANDROID_LOG_WARN   , TAG,__VA_ARGS__)
#define LOGE(TAG,...) __android_log_print(ANDROID_LOG_ERROR  , TAG,__VA_ARGS__)

#include <jni.h>
#include <android/log.h>
#include <android/bitmap.h>

#ifdef __cplusplus
extern "C" {
#endif

// This buffer is used for the 2 frame circular buffer that is passed on to grab frame.
struct buffer { 
	void *start;
	size_t length;
};

//------------------------------------------------------------------------------
//  This is the main function that populates the circular buffer with the next
//  frame of images. This also does the FCV colour space conversions.
//------------------------------------------------------------------------------ 
static void process_image(const void *p);

//------------------------------------------------------------------------------
//  The following functions are for initialization and management of the V4L2 
//  buffer. The following functions shouldn't need to be modified for other
//  applications. 
//------------------------------------------------------------------------------ 
static int read_frame(void);
static void errno_exit(const char *s);
static int xioctl(int fd, int request, void *arg);
static void stop_capturing(void);
static void start_capturing(void);
static void uninit_device(void);
static void init_mmap(void);
static void init_device(void);
static void close_device(void);
static void open_device(void);

//------------------------------------------------------------------------------
//  The following non-static functions are the exposed functions for native calls
//  or render calls. If you are spawning a seperate thread in JNI for the camera
//  then the exposed init, close, and main camera functions are necessary.
//------------------------------------------------------------------------------ 
int grabFrame(char* p, int size);
int grabYFrame(char* p, int size);
void initCamera(int* w, int*h);
void closeCamera(void);
void cameraMain(void);

//------------------------------------------------------------------------------
//  JNI Interface
//------------------------------------------------------------------------------ 
JNIEXPORT void JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_initCamera (JNIEnv * env, jobject obj);
JNIEXPORT void JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_cameraMain (JNIEnv * env, jobject obj);
JNIEXPORT void JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_closeCamera (JNIEnv * env, jobject obj);

#ifdef __cplusplus
}
#endif
#endif
