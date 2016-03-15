package com.roboclub.robobuggy.main;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.orsoncharts.util.json.JSONObject;




// add JSON parser for RobobuggyConfigFile

/**
 * Class used to store the system configuration values
 */
public final class  RobobuggyConfigFile {

    //Library info
    public static final String ALICE_LIBRARY_VERSION = "1.0.0";
	public static final String RBSM_HEADER_FILE_LOCATION = "../../../arduino_src/lib_avr/rbserialmessages/rbsm_headers.txt";

	// for what messages should be printed to the console
	public static final RobobuggyMessageLevel REPORTING_LEVEL = RobobuggyMessageLevel.NOTE;
	// for what messages should be printed to the console
	public static final int GRAPH_LENGTH = 100;

	// default logging state, should the buggy start logging as soon as this
	// program is started
	public static final boolean LOGGING = true;
	public static final String LOG_FILE_LOCATION = "logs";
	public static final String LOG_FILE_NAME = "sensors";

	//Autonomous controls
	public static final boolean AUTONOMOUS_DEFAULT = false;
	public static final int RBSM_COMMAND_PERIOD = 50;

	// sensor default settings (true for on false for off
	public static boolean IMU_ENABLED = true;
	public static boolean GPS_ENABLED = true;
	public static boolean ENCODER_ENABLED = true;
	public static boolean VISION_SYSTEM_ENABLED = true;

	//sensor com ports
	public static String COM_PORT_IMU = "/dev/tty.usbserial-A6026UA0";

	public static String COM_PORT_GPS = "/dev/tty.usbmodem142141";
	public static String COM_PORT_RBSM = "/dev/tty.usbmodem142111";
	public static String PORT_VISION ="";

	// iff false, connect to serial sensors 
	public static final boolean DATA_PLAY_BACK = true;

	public static String WAYPOINT_SOURCE_LOG_FILE ="2016-02-20-06-50-45/sensors_2016-02-20-06-50-45.txt";
	public static String PLAY_BACK_SOURCE_FILE = "logs/2016-03-12-17-37-04/sensors_2016-03-12-17-37-04.txt";
	
	//is where values from this file are saved so that they can be updated between runs
	public static String CONFIG_FILE ="THIS FEATURE HAS NOT BEEN IMPLEMENTED YET";
	
	public static void saveConfigFile(){
		JsonObject settings = new JsonObject();
		settings.addProperty("COM_PORT_IMU", COM_PORT_IMU);
		settings.addProperty("COM_PORT_RBSM", COM_PORT_RBSM);
		settings.addProperty("PORT_VISION", PORT_VISION);
		settings.addProperty("COM_PORT_GPS", COM_PORT_GPS);
		settings.addProperty("WAYPOINT_SOURCE_LOG_FILE", WAYPOINT_SOURCE_LOG_FILE);
		settings.addProperty("PLAY_BACK_SOURCE_FILE", PLAY_BACK_SOURCE_FILE);
		
		//TODO add other settings as they are created 
	}
	
	public static boolean loadConfigFile(){
		//TODO
		return false;
	}
	
	
}
