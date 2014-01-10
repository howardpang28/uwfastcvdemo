package com.example.usbfastdemo;

import com.example.usbfastdemo.*;
import com.example.usbfastdemo.R;
import android.content.res.AssetManager;
import android.graphics.Bitmap;

public class USBFastLib {
   static {
      System.loadLibrary( "usbfastdemo" );
   };

   private USBFastActivity _main;

   public USBFastLib(USBFastActivity main){
		_main = main;
   }

   public static native void initCamera();
   public static native void cameraMain();
   public static native void closeCamera();
   public static native void drawMain(Bitmap bm);
    public static native int isDirty();

   public static native void initOpenGL( int w, int h);
   public static native void step( float fElapsedTime);
   public static native void createAssetManager( AssetManager assetManager );  
   
}
