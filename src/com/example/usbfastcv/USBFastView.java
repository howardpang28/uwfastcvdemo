package com.example.usbfastdemo;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.Log;
import android.view.View;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;
import android.os.SystemClock;


public class USBFastView extends GLSurfaceView {
    public USBFastView( Context context ) 
    {
        super( context );

        // Create an OpenGL ES 2.0 context
        setEGLContextClientVersion( 2 );

        // Set the renderer associated with this view
        setRenderer( new USBFastRenderer() );
    }

    private static class USBFastRenderer implements GLSurfaceView.Renderer 
    {
        private long m_nLastTime;

        public void onDrawFrame( GL10 gl )
        {
            // calculate elapsed time
            if( m_nLastTime == 0 )
                m_nLastTime = SystemClock.elapsedRealtime();

            long nCurrentTime = SystemClock.elapsedRealtime();
            long nElapsedTime = nCurrentTime - m_nLastTime;
            float fElapsedTime = nElapsedTime / 1000.0f;
            m_nLastTime = nCurrentTime;

            USBFastLib.step( fElapsedTime );
        }

        public void onSurfaceChanged( GL10 gl, int width, int height )
        {
            USBFastLib.initOpenGL( width, height );
        }

        public void onSurfaceCreated( GL10 gl, EGLConfig config )
        {

        }
    }
}
