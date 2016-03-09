package com.roboclub.robobuggy.main;




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
	public static final boolean IMU_ENABLED = true;
	public static final boolean GPS_ENABLED = true;
	public static final boolean ENCODER_ENABLED = true;
	public static final boolean VISION_SYSTEM_ENABLED = true;

	//sensor com ports
	public static  final String COM_PORT_IMU = "/dev/tty.usbserial-A6026UA0";

	public static  final String COM_PORT_GPS = "/dev/tty.usbmodem142141";
	public static  final String COM_PORT_RBSM = "/dev/tty.usbmodem142111";

	// iff false, connect to serial sensors 
	public static final boolean DATA_PLAY_BACK = false;

	public static final String WAYPOINT_SOURCE_LOG_FILE ="2016-02-20-06-50-45/sensors_2016-02-20-06-50-45.txt";

}
