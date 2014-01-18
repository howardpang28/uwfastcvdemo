/*
 *  External USB webcam demo through V4L2 interface.
 *
 *  This file manages grabbing the frames from the USB camera calling the
 *  grabFrame() from capture.cpp. Once a frame is grabbed, it will draw it
 *  on the screen and also implement all the FastCV algorithms.
 *
 *  This program is property of University of Waterloo and can be used and 
 *  distributed without restrictions.
 *
 *  File: render.cpp
 *  Author: Howard Pang
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <fastcv/fastcv.h>
#include <fcntl.h>              /* low-level i/o */
#include <pthread.h>

#include "render.h"
#include "capture.h"

#define LOG_TAG    "USBFastDemoJNI"

// The mutex is needed for the circular buffer used in grab frame
pthread_mutex_t mutexDirty;
int dirty = 0;
int dump_fd;
int low, high;

//------------------------------------------------------------------------------
//  This function is for a quick swap. This is used in Brensenham's algorithm.
//------------------------------------------------------------------------------
static void swap( int& a, int& b)
{
    int tmp = a;
    a = b;
    b = tmp;
}

//------------------------------------------------------------------------------
//  Draw the line (x1,y1) to (x2,y2) passed through fixed array Points. Pass the
//  maximum number of points that the points array can handle.
//------------------------------------------------------------------------------
static int bresenham( Point* points, double x1, double y1, double x2, double y2, int max)
{
    Point pnt1( round( x1), round( y1));
    Point pnt2( round( x2), round( y2));

    // make sure that the line is not steep
    bool steep = (abs(pnt2.y - pnt1.y) > abs(pnt2.x - pnt1.x));
    if (steep) {
        swap( pnt1.x, pnt1.y);
        swap( pnt2.x, pnt2.y);
    }

    // make sure that the line only goes right -> left
    if ( pnt1.x > pnt2.x) {
        swap( pnt1.x, pnt2.x);
        swap( pnt1.y, pnt2.y);
    }
   
    int dx = abs(pnt2.x - pnt1.x);
    int dy = abs(pnt2.y - pnt1.y);
   
    int error = dx >> 1;
    int ystep = (pnt1.y < pnt2.y) ? 1 : -1;	// 1 if going up, -1 if down
   
    int y = pnt1.y;
    int curPnt = 0;
    for (int x = pnt1.x; x < pnt2.x; x++) {
        if ( steep) points[curPnt] = Point(y, x);
        else	    points[curPnt] = Point(x, y);
        curPnt++;
        if (curPnt == max)
            return curPnt;

        error -= dy;
        if (error < 0) {
	        y += ystep;
	        error += dx;
        }
    }
    return curPnt;
}

//------------------------------------------------------------------------------
//  All of the FCV processing of the frame happens in this function.
//------------------------------------------------------------------------------
void processFrame( char* y, char* rgba, int width, int height, int bpp)
{
    //--------------------------------
    // Perform hough line detection
    //--------------------------------
    /*uint32_t maxLines = 15, numLines;
    fcvLine detLines[maxLines];
    fcvHoughLineu8 ( (uint8_t*)y, width, height, 0, 1.0, maxLines, &numLines, detLines);

    int maxPnts = 500, numPnts;
    Point points[maxPnts];
    for (unsigned int i = 0; i < numLines; i++) {
        numPnts = bresenham( points, detLines[i].start.x, detLines[i].start.y, detLines[i].end.x, detLines[i].end.y, maxPnts );
        //LOGI(LOG_TAG, "numPnts = %i", numPnts);
        //LOGI(LOG_TAG, "(%i,%i) to (%i,%i)", (int)(detLines[i].start.x),(int)(detLines[i].start.y),
        //                                    (int)(detLines[i].end.x),  (int)(detLines[i].end.y));
        for( int j = 0; j < numPnts; j++) {
        //LOGI(LOG_TAG, "red at (%i,%i)", points[j].x, points[j].y);
            int pIdx = points[j].x * bpp + points[j].y * width * bpp;
            //rgba[pIdx] = 255;
            //rgba[pIdx + 1] = 0;
            //rgba[pIdx + 2] = 0;
            //rgba[pIdx + 3] = 255;
        }
    }*/

    //--------------------------------
    // Perform corner detection
    //--------------------------------
    /*uint32_t numCorners, maxCorners = 1000;
    uint32_t* corners = (uint32_t*)fcvMemAlloc( maxCorners*4*2, 16);
    fcvCornerFast9u8 ( (uint8_t*)y, scaledWidth, scaledHeight, 0, 25, 0, corners, maxCorners, &numCorners);

    LOGI(LOG_TAG, "Found %i coners", numCorners);
    for( unsigned int j = 0; j < numCorners; j+=2) {
        uint32_t pIdx = corners[j] * bpp + corners[j+1] * width * bpp;
        if ((int)pIdx < scaledWidth*scaledHeight*4) {
            rgba[pIdx] = 255;
            rgba[pIdx + 1] = 0;
            rgba[pIdx + 2] = 0;
            rgba[pIdx + 3] = 255;
        }
    }
    fcvMemFree( corners);*/

    //--------------------------------
    // Perform canny edge detection
    //--------------------------------
    int scaledWidth = width/2;
    int scaledHeight = height/2;

	uint8_t* scaledBuf = (uint8_t*)fcvMemAlloc( scaledWidth*scaledHeight, 16);
	uint8_t* filteredBuf = (uint8_t*)fcvMemAlloc( scaledWidth*scaledHeight, 16);

    // scale down the image and then apply 5x5 gaussian filter
    fcvScaleDownBy2u8( (uint8_t*)y, width, height, scaledBuf );
    fcvFilterGaussian5x5u8( scaledBuf, scaledWidth, scaledHeight, filteredBuf, 1 );

    uint8_t*  cannyBuf = (uint8_t*)fcvMemAlloc(scaledWidth * scaledHeight, 16);
    fcvFilterCanny3x3u8 ( filteredBuf, scaledWidth, scaledHeight, cannyBuf, 12, 14);
    write(dump_fd, cannyBuf, scaledWidth * scaledHeight);

    /*for (int j = 0; j < scaledHeight*scaledWidth; j++) {
        int idx = j*4;  // upscale back
        if (cannyBuf[j] != 0) {
            rgba[idx*4] = 0;
            rgba[idx*4+1] = 0;
            rgba[idx*4+2] = 0;
            rgba[idx*4+3] = 255;
        } else {
            rgba[idx*4+3] = 0;
        }
    }*/

    fcvMemFree (cannyBuf);
    fcvMemFree(scaledBuf);
    fcvMemFree(filteredBuf);
}

//------------------------------------------------------------------------------
//  Draw the line (x1,y1) to (x2,y2) passed through fixed array Points. Pass the
//  maximum number of points that the points array can handle.
//------------------------------------------------------------------------------
JNIEXPORT void JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_drawMain (JNIEnv * env, jobject obj, jobject bitmap)
{
   LOGI(LOG_TAG, "inside drawMain"); 
    dump_fd = open("/sdcard/dumpcan.yuv", O_RDWR|O_CREAT, 0777);

    AndroidBitmapInfo info;
    AndroidBitmap_getInfo(env,bitmap,&info);
    int width = info.width, height = info.height, bbp = info.stride / info.width;
    LOGI(LOG_TAG, "Render buffer width: %d height: %d bbp: %d",width,height,bbp);

  	bitmap = env->NewGlobalRef(bitmap);
	char* frameBufferY = (char*)fcvMemAlloc( width*height, 16);
	char* frameBufferRGBA = (char*)fcvMemAlloc( width*height*bbp, 16);    

    while(1) {
        int ret = AndroidBitmap_lockPixels(env,bitmap,(void**)&frameBufferRGBA);
        if(ret == ANDROID_BITMAP_RESUT_SUCCESS){
            ret = 1;
            while (ret) ret = grabFrame(frameBufferRGBA, width*height*bbp);

            AndroidBitmap_unlockPixels(env,bitmap);

  	        pthread_mutex_lock( &mutexDirty );
            dirty = 1;
  	        pthread_mutex_unlock( &mutexDirty );

        }
        else LOGE(LOG_TAG,"Lock pixel failed: %d",ret);
    }
    fcvMemFree(frameBufferRGBA);
    fcvMemFree(frameBufferY);
}

//------------------------------------------------------------------------------
//  This function is called by a Java thread that checks if there is a new frame
//  that needs to be drawn on the screen. If there is a new frame, then update
//  The screen by invoking Render in Java.
//------------------------------------------------------------------------------
JNIEXPORT int JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_isDirty (JNIEnv * env, jobject obj)
{
   pthread_mutex_lock( &mutexDirty );
   if (dirty) {
        dirty = 0;
  	    pthread_mutex_unlock( &mutexDirty );
        return 1;
   } else {
        pthread_mutex_unlock( &mutexDirty );
        return 0;
   }
}
