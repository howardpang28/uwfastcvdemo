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

#include "render.h"
#include "capture.h"

#define LOG_TAG    "USBFastDemoJNI"

int dirty = 0;

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
void processFrame( char* y, char* rgba, int width, int height, int size)
{
        // Perform hough line detection on the y data
        uint32_t maxLines = 15;
        uint32_t ndetLines;
        fcvLine detLines[maxLines];
        fcvHoughLineu8 ( (uint8_t*)y, width, height, 0, 1.0, maxLines, &ndetLines, detLines);

        int maxPnts = 500, numPnts;
        Point points[maxPnts];
        for (unsigned int i = 0; i < ndetLines; i++) {
            numPnts = bresenham( points, detLines[i].start.x, detLines[i].start.y, detLines[i].end.x, detLines[i].end.y, maxPnts );
            //LOGI(LOG_TAG, "numPnts = %i", numPnts);
            //LOGI(LOG_TAG, "(%i,%i) to (%i,%i)", (int)(detLines[i].start.x),(int)(detLines[i].start.y),
            //                                    (int)(detLines[i].end.x),  (int)(detLines[i].end.y));
            for( int j = 0; j < numPnts; j++) {
            //LOGI(LOG_TAG, "red at (%i,%i)", points[j].x, points[j].y);                
                int pIdx = points[j].x * 4 + points[j].y * width * 4;
                rgba[pIdx] = 255;
                rgba[pIdx + 1] = 0;
                rgba[pIdx + 2] = 0;
                rgba[pIdx + 3] = 255; 
            }
        }
}

//------------------------------------------------------------------------------
//  Draw the line (x1,y1) to (x2,y2) passed through fixed array Points. Pass the
//  maximum number of points that the points array can handle.
//------------------------------------------------------------------------------
JNIEXPORT void JNICALL 
    Java_com_example_usbfastdemo_USBFastLib_drawMain (JNIEnv * env, jobject obj, jobject bitmap)
{
   LOGI(LOG_TAG, "inside drawMain"); 
   //int dump_fd2 = open("/sdcard/dumprgb.yuv", O_RDWR|O_CREAT, 0777);    

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
            while (ret) ret = grabYFrame(frameBufferY, width*height);

            ret = 1;
            while (ret) ret = grabFrame(frameBufferRGBA, width*height*bbp);

            processFrame(frameBufferY, frameBufferRGBA, width, height, bbp);
	        
            //write(dump_fd2, frameBufferRGBA, width*height*bbp);
            
            AndroidBitmap_unlockPixels(env,bitmap);
            dirty = 1;            
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
   if (dirty) {
        dirty = 0;
        return 1;
   } else {
        return 0;
   }
}
