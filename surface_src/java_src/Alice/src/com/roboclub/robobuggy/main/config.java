package com.roboclub.robobuggy.main;

// where all system configuration values should be placed
// add jason parser for config

public class  config{
//port index of the front camera 
public static final int FRONT_CAM_INDEX = 0;

//port index of the rear (pushbar camera)
public static final int REAR_CAM_INDEX = 1;

//location of the executable that should be run for the camera sub system 
public static final String VISION_SYSTEM_EXECUTABLE_LOCATION = "C:\\Users\\abc\\buggy-log\\VisionSystem.exe";

//default logging state, should the buggy start logging as soon as this program is started
public static boolean LOGGING_DEFAULT = true;

//default running state, should the buggy program start running as soon as this program is started
public static boolean ACTIVE_DEFAULT = false;

public static boolean AUTONOMUS_DEFAULT = false;

//sensor default settings (true for on false for off)

public static boolean DRIVE_DEFAULT = false;
public static boolean IMU_DEFAULT = true;
public static boolean GPS_DEFAULT = true;
public static boolean ENCODER_DEFAULT = true;
public static boolean VISION_SYSTEM_DEFAULT = true;

//number of times that we will allow for the brakes to be deployed and still have the buggy run
public static byte BRAKES_PER_FULL_PRESSURE = 4;  

public static boolean GUI_ON_DEFAULT = true;
public static boolean DATA_PLAY_BACK_DEFAULT = false;//if false then try to read from live sensors 


//current status values 
public static boolean GUI_ON;
public static boolean active;
public static boolean logging;


//internal reference of this config so that it can fit the factory pattern
private static config instance;

public static config getInstance(){
	if(instance == null)
	{
		instance = new config();
		//sets defaults
		logging = config.LOGGING_DEFAULT;
		GUI_ON = GUI_ON_DEFAULT;
		active = ACTIVE_DEFAULT;
	}
	return instance;
}


}
