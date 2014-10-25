package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.main.config;

/**
 * 
 * @author Kevin Brennan
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class CLCamera {
	// camera color mode
    public static int CLEYE_MONO_PROCESSED	= 0;
    public static int CLEYE_COLOR_PROCESSED	= 1;
    public static int CLEYE_MONO_RAW            = 2;
    public static int CLEYE_COLOR_RAW           = 3;
    public static int CLEYE_BAYER_RAW           = 4;

    // camera resolution
    public static int CLEYE_QVGA		= 0;
    public static int CLEYE_VGA			= 1;

    // camera sensor parameters
    public static int CLEYE_AUTO_GAIN 		= 0;  	// [0, 1]
    public static int CLEYE_GAIN		= 1;	// [0, 79]
    public static int CLEYE_AUTO_EXPOSURE	= 2;    // [0, 1]
    public static int CLEYE_EXPOSURE		= 3;    // [0, 511]
    public static int CLEYE_AUTO_WHITEBALANCE	= 4;	// [0, 1]
    public static int CLEYE_WHITEBALANCE_RED	= 5;	// [0, 255]
    public static int CLEYE_WHITEBALANCE_GREEN	= 6;   	// [0, 255]
    public static int CLEYE_WHITEBALANCE_BLUE	= 7;    // [0, 255]
    // camera linear transform parameters
    public static int CLEYE_HFLIP		= 8;    // [0, 1]
    public static int CLEYE_VFLIP		= 9;    // [0, 1]
    public static int CLEYE_HKEYSTONE		= 10;   // [-500, 500]
    public static int CLEYE_VKEYSTONE		= 11;   // [-500, 500]
    public static int CLEYE_XOFFSET		= 12;   // [-500, 500]
    public static int CLEYE_YOFFSET		= 13;   // [-500, 500]
    public static int CLEYE_ROTATION		= 14;   // [-500, 500]
    public static int CLEYE_ZOOM		= 15;   // [-500, 500]
    // camera non-linear transform parameters
    public static int CLEYE_LENSCORRECTION1	= 16;	// [-500, 500]
    public static int CLEYE_LENSCORRECTION2	= 17;	// [-500, 500]
    public static int CLEYE_LENSCORRECTION3	= 18;	// [-500, 500]
    public static int CLEYE_LENSBRIGHTNESS	= 19;	// [-500, 500]

    native static int CLEyeGetCameraCount();
    native static String CLEyeGetCameraUUID(int index);
    native static int CLEyeCreateCamera(int cameraIndex, int mode, int resolution, int framerate);
    native static boolean CLEyeDestroyCamera(int cameraIndex);
    native static boolean CLEyeCameraStart(int cameraInstance);
    native static boolean CLEyeCameraStop(int cameraInstance);
    native static boolean CLEyeSetCameraParameter(int cameraInstance, int param, int val);
    native static int CLEyeGetCameraParameter(int cameraInstance, int param);
    native static boolean CLEyeCameraGetFrame(int cameraInstance, int[] imgData, int waitTimeout);
    
	private int cameraInstance;
	
	public CLCamera(int cameraId, int framerate) {
		try {
			System.load(config.clEyeDllPath);
		} catch (Exception e) {
			System.out.println("Failed To Load JNI wrapper for CL Eye Multicam");
			return;
		}
		
		if (!createCamera(cameraId, CLEYE_COLOR_RAW , CLEYE_QVGA, framerate)) {
			System.out.println("Failed to open camera capture for camera: " + cameraId);
			return;
		};
		
		dispose();
	}
	
	public void dispose()
    {
        stopCamera();
        destroyCamera();
    }
    public boolean createCamera(int cameraIndex, int mode, int resolution, int framerate)
    {
        cameraInstance = CLEyeCreateCamera(cameraIndex, mode, resolution, framerate);
        return cameraInstance != 0;
    }
    public boolean destroyCamera()
    {
        return CLEyeDestroyCamera(cameraInstance);
    }
    public boolean startCamera()
    {
        return CLEyeCameraStart(cameraInstance);
    }
    public boolean stopCamera()
    {
        return CLEyeCameraStop(cameraInstance);
    }
    public boolean getCameraFrame(int[] imgData, int waitTimeout)
    {
        return CLEyeCameraGetFrame(cameraInstance, imgData, waitTimeout);
    }
    public boolean setCameraParam(int param, int val)
    {
        return CLEyeSetCameraParameter(cameraInstance, param, val);
    }
    public int getCameraParam(int param)
    {
        return CLEyeGetCameraParameter(cameraInstance, param);
    }
}