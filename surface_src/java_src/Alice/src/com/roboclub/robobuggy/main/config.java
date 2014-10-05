package com.roboclub.robobuggy.main;

//where all system configuration values should be placed

public class  config{
//port index of the front camera 
public static final int FRONT_CAM_INDEX = 0;

//port index of the rear (pushbar camera)
public static final int REAR_CAM_INDEX = 1;

//location of the executable that should be run for the camera sub system 
public static final String VISION_SYSTEM_EXECUTABLE_LOCATION = "C:\\Users\\abc\\buggy-log\\VisionSystem.exe";

//default logging state, should the buggy start logging as soon as this program is started
public static boolean LOGGING_DEFAULT = false;

//default running state, should the buggy program start running as soon as this program is started
public static boolean RUNNING_DEFAULT = true;

//sensor default settings (true for on false for off)
public static boolean IMU_DEFAULT = true;
public static boolean GPS_DEFAULT = true;
public static boolean ENCODER_DEFAULT = true;
public static boolean VISION_SYSTEM_DEFAULT = true;
public static boolean COMMANDED_ANGLE_DEFAULT = true;

//number of times that we will allow for the brakes to be deployed and still have the buggy run
public static byte BRAKES_PER_FULL_PRESSURE = 4;  

//internal reference of this config so that it can fit the factory pattern
private static config instance;

public static config getInstance(){
	if(instance == null)
	{
		instance = new config();
	}
	return instance;
}


}
