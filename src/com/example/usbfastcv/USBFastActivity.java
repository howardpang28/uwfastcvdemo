/*==============================================================================
            Copyright (c) 2013 University of Waterloo
            All Rights Reserved.
            Author: Howard Pang
==============================================================================*/

package com.example.usbfastdemo;

import java.nio.ShortBuffer;

import com.example.usbfastdemo.R;
import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.opengl.GLSurfaceView;
import android.view.Window;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.ToggleButton;
import android.widget.TextView;
import android.content.res.AssetManager;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Rect;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.widget.ImageView;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

public class USBFastActivity extends Activity
{
    static AssetManager sAssetManager;
    USBFastView         mView;
    USBFastLib          mLib; 

    private static final String TAG  = "USBFastDemo";
    private Thread captureT = null;   
    private Thread drawT = null;
    private Thread renderT = null;

    private Bitmap _bmSurface;
    private SurfaceView _surface;
    private SurfaceHolder _holder;
    private int height = 240;
    private int width = 320;

    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate( savedInstanceState );
		setContentView(R.layout.main);

        _surface = (SurfaceView) findViewById(R.id.renderingSurface);
		_holder = _surface.getHolder();

        // ARGB is actually RGBA
		_bmSurface = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);


        // init the Native interface
        mLib = new USBFastLib(this);
        //mLib.initNative(); 

        // Pass the asset manager to the native code
        //sAssetManager = getAssets();
        //mLib.createAssetManager( sAssetManager );
        
        // Create our view for OpenGL rendering
        //mView = new USBFastView( getApplication() );
        
        //setContentView( mView );

        mLib.initCamera();
        captureT = new Thread(new CaptureThread());
	    captureT.start();

        drawT = new Thread( new DrawThread());
        drawT.start();

        renderT = new Thread( new RenderThread());
        renderT.start();
   }
   
   @Override
   protected void onResume()
   {
      super.onResume();
      //mView.onResume();
   }

   	class CaptureThread implements Runnable{
		@Override
		public void run() {
            mLib.cameraMain();
        }
    }

    class DrawThread implements Runnable{
		@Override
		public void run() {
            mLib.drawMain(_bmSurface);
        }
    }

    class RenderThread implements Runnable{
		@Override
		public void run() {
            while(true) {
                if (mLib.isDirty() == 1) {
                    Render();
                }
            }
        }
    }

	@Override
	protected void onDestroy() {
		Log.d(TAG,"Activity stoppedd");
        try {
	        captureT.join();
  		} catch (InterruptedException e) {
		}
        mLib.closeCamera();
        
		super.onStop();
	}

	public void Render(){
		if(!_holder.getSurface().isValid()){
			return;
		}
		Canvas c = _holder.lockCanvas();
		c.drawBitmap(_bmSurface, new Rect(0,0,_bmSurface.getWidth(),_bmSurface.getHeight()), 
				new Rect(0,0,_surface.getWidth(), _surface.getHeight()), null);
		_holder.unlockCanvasAndPost(c);
	}
}
