package com.roboclub.robobuggy.main;

import java.lang.reflect.Field;
import java.util.Arrays;

// where all system configuration values should be placed
// add jason parser for RobobuggyConfigFile

public final class  RobobuggyConfigFile {

	public static final RobobuggyMessageLevel REPORTING_LEVEL = RobobuggyMessageLevel.NOTE; // for what messages should be printed to the console


	public static final String LOG_FILE_LOCATION = "logs";
	public static final String LOG_STOP_MESSAGE = "STOP_LOGGING";

	// default logging state, should the buggy start logging as soon as this
	// program is started
	public static final boolean LOGGING_DEFAULT = true;

	// default running state, should the buggy program start running as soon as
	// this program is started
	public static final boolean ACTIVE_DEFAULT = false;

	public static final boolean AUTONOMOUS_DEFAULT = false;

	// sensor default settings (true for on false for off)
	public static  final boolean DRIVE_DEFAULT = true;
	public static  final boolean IMU_DEFAULT = true;
	public static  final boolean GPS_DEFAULT = true;
	public static  final boolean ENCODER_DEFAULT = true;
	public static  final boolean VISION_SYSTEM_DEFAULT = true;
	
	//sensor com ports
	public static  final String COM_PORT_IMU = "COM6";
	public static  final String COM_PORT_GPS_INTEGRATED = "COM7";
	public static  final String COM_PORT_GPS_STANDALONE = "COM8";
	public static  final String COM_PORT_ENCODER = "/dev/tty.usbmodem14231";
	public static  final String COM_PORT_DRIVE_CONTROL = "/dev/tty.usbmodem14231";
	
	// number of times that we will allow for the brakes to be deployed and
	// still have the buggy run
	public static final byte BRAKES_PER_FULL_PRESSURE = 4;
	public static final boolean GUI_ON_DEFAULT = true;
	// iff false, connect to serial sensors 
	public static final boolean DATA_PLAY_BACK_DEFAULT = true;
	
	// current status values
	public static boolean GUI_ON;
	public static boolean IS_GUI_CURRENTLY_LOGGING;
	public static boolean logging;

	// internal reference of this RobobuggyConfigFile so that it can fit the factory pattern
	private static RobobuggyConfigFile instance;

	public static RobobuggyConfigFile getInstance() {
		if (instance == null) {
			instance = new RobobuggyConfigFile();
			// sets defaults
			logging = RobobuggyConfigFile.LOGGING_DEFAULT;
			GUI_ON = GUI_ON_DEFAULT;
			IS_GUI_CURRENTLY_LOGGING = ACTIVE_DEFAULT;
		}
		return instance;
	}

	//includes all of the jni libraries that we need to be able to use all of our libraries
	public static boolean setupJNI() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException{
	       final String PATH_TO_ADD =  "library";
	       final Field usrPathsField = ClassLoader.class.getDeclaredField("usr_paths");
	       usrPathsField.setAccessible(true);

	       //get array of paths
	       final String[] paths = (String[])usrPathsField.get(null);
	       //check if the path to add is already present
	       for(final String path : paths) {
	           if(path.equals(PATH_TO_ADD)) {
	               return true;
	           }
	       }

	       //add the new path
	       final String[] newPaths = Arrays.copyOf(paths, paths.length + 1);
	       newPaths[newPaths.length-1] = PATH_TO_ADD;
	       usrPathsField.set(null, newPaths);
	       
	       return true;
	}

}
