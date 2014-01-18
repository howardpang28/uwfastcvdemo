#ifndef _RENDER_
#define _RENDER_

/*
 *  External USB webcam demo through V4L2 interface.
 *
 *  This file definition manages grabbing the frames from the USB camera calling the
 *  grabFrame() from capture.cpp. Once a frame is grabbed, it will draw it
 *  on the screen and also implement all the FastCV algorithms.
 *
 *  This program is property of University of Waterloo and can be used and 
 *  distributed without restrictions.
 *
 *  File: render.h
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

//------------------------------------------------------------------------------
//  This struct is used for the Brensenham's occupancy grid algorithm.
//------------------------------------------------------------------------------
struct Point {
    int x;
    int y;
    Point() : x(0), y(0){}
    Point( int x, int y) : x(x), y(y){}
};

//------------------------------------------------------------------------------
//  The following two functions are used for filling up the bitmap based on the 
//  fcv outputs. The fcv algotihm outputs are generally given in terms of start
//  and end positions. Thus it is necessary to run Brensenham's occupancy grid
//  algorithm.
//------------------------------------------------------------------------------
static void swap( int& a, int& b);
static int bresenham( Point* points, double x1, double y1, double x2, double y2, int max);


//------------------------------------------------------------------------------
//  All of the FCV processing of the frame happens in this function.
//------------------------------------------------------------------------------
void processFrame( char* y, char* rgba, int width, int height, int bpp);

//------------------------------------------------------------------------------
//  JNI Interface
//------------------------------------------------------------------------------ 
JNIEXPORT void JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_drawMain (JNIEnv * env, jobject obj, jobject bitmap);
JNIEXPORT int JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_isDirty (JNIEnv * env, jobject obj);

#ifdef __cplusplus
}
#endif
#endif
