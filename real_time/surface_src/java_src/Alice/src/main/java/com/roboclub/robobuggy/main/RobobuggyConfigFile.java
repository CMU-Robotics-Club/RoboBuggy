package com.roboclub.robobuggy.main;


// add jason parser for RobobuggyConfigFile
/**
 * Class used to store the system configuration values
 */
public final class  RobobuggyConfigFile {

	// for what messages should be printed to the console
	public static final RobobuggyMessageLevel REPORTING_LEVEL = RobobuggyMessageLevel.NOTE;

	public static final String LOG_FILE_LOCATION = "logs";
	public static final String LOG_STOP_MESSAGE = "STOP_LOGGING";

	// default logging state, should the buggy start logging as soon as this
	// program is started
	public static final boolean LOGGING = true;

	public static final boolean AUTONOMOUS_DEFAULT = false;

	// sensor default settings (true for on false for off
	public static final boolean IMU_ENABLED = true;
	public static final boolean GPS_ENABLED = true;
	public static final boolean ENCODER_ENABLED = true;
	public static final boolean VISION_SYSTEM_ENABLED = true;
	
	//sensor com ports
	public static  final String COM_PORT_IMU = "COM6";
	public static  final String COM_PORT_GPS = "COM12";
	public static  final String COM_PORT_RBSM = "COM3";
	
	// iff false, connect to serial sensors 
	public static final boolean DATA_PLAY_BACK = true;

}