package com.roboclub.robobuggy.main;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.Arrays;

import com.orsoncharts.util.json.JSONObject;
import com.orsoncharts.util.json.parser.JSONParser;
import com.orsoncharts.util.json.parser.ParseException;

// where all system configuration values should be placed
// add jason parser for RobobuggyConfigFile

public class RobobuggyConfigFile {
	// port index of the front camera
	public static int FRONT_CAM_INDEX = 2;
	
	public static final RobobuggyMessageLevel REPORTING_LEVEL = RobobuggyMessageLevel.NOTE;

	// port index of the rear (pushbar camera back)
	public static int REAR_CAM_INDEX = 0;
	
	//port index of the overlook (pushbar camera front)
	public static int OVERLOOK_CAM_INDEX = 1;//should be number 4

	// location of the executable that should be run for the camera sub system
	public static String VISION_SYSTEM_EXECUTABLE_LOCATION = "C:\\Users\\Robot\\Documents\\GitHub\\RoboBuggy\\surface_src\\VisionSystem\\Debug\\VisionSystem.exe";
	// "C:\\Users\\abc\\buggy-log\\VisionSystem.exe";

	public static String LOG_FILE_LOCATION = "logs";
	public static String LOG_STOP_MESSAGE = "STOP_LOGGING";

	// default logging state, should the buggy start logging as soon as this
	// program is started
	public static boolean LOGGING_DEFAULT = true;

	// default running state, should the buggy program start running as soon as
	// this program is started
	public static boolean ACTIVE_DEFAULT = false;

	public static boolean AUTONOMOUS_DEFAULT = false;

	// sensor default settings (true for on false for off)
	public static  boolean DRIVE_DEFAULT = true;
	public static  boolean IMU_DEFAULT = true;
	public static  boolean GPS_DEFAULT = true;
	public static  boolean ENCODER_DEFAULT = true;
	public static  boolean VISION_SYSTEM_DEFAULT = true;
	
	//sensor com ports
	public static  String COM_PORT_IMU = "COM6";
	public static  String COM_PORT_GPS_INTEGRATED = "COM7";
	public static  String COM_PORT_GPS_STANDALONE = "COM8";
	public static  String COM_PORT_ENCODER = "/dev/tty.usbmodem14231";
	public static  String COM_PORT_DRIVE_CONTROL = "/dev/tty.usbmodem14231";
	
	//for turning indvidual cams on and off
	public static  boolean FRONT_CAM_ON = false;
	public static  boolean REAR_CAM_ON = false;
	public static  boolean OVERLOOK_CAM_ON = false;
	
	
	// number of times that we will allow for the brakes to be deployed and
	// still have the buggy run
	public static byte BRAKES_PER_FULL_PRESSURE = 4;

	public static boolean GUI_ON_DEFAULT = true;
	// iff false, connect to serial sensors 
	public static final boolean DATA_PLAY_BACK_DEFAULT = false;
	
	// current status values
	public static boolean GUI_ON;
	public static boolean IS_GUI_CURRENTLY_LOGGING;
	public static boolean logging;

	public static String clEyeDllPath = "C://CL-Eye Platform SDK//Bin//CLEyeMulticam.dll";

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

	@SuppressWarnings("unchecked")
	public static void Set(String filename) {
		System.out.println("Setting configuration parameters from file: "
				+ filename);

		try {
			JSONParser parser = new JSONParser();
			JSONObject obj = (JSONObject) parser
					.parse(new FileReader(filename));

			FRONT_CAM_INDEX = (int) obj.getOrDefault("FRONT_CAM_INDEX",
					RobobuggyConfigFile.FRONT_CAM_INDEX);
			REAR_CAM_INDEX = (int) obj.getOrDefault("BACK_CAM_INDEX",
					RobobuggyConfigFile.REAR_CAM_INDEX);
			OVERLOOK_CAM_INDEX = (int) obj.getOrDefault("OVERLOOK_CAM_INDEX", RobobuggyConfigFile.OVERLOOK_CAM_INDEX);
			VISION_SYSTEM_EXECUTABLE_LOCATION = (String) obj.getOrDefault(
					"VISION_SYSTEM_EXECUTABLE_LOCATION",
					RobobuggyConfigFile.VISION_SYSTEM_EXECUTABLE_LOCATION);
			LOG_FILE_LOCATION = (String) obj.getOrDefault("LOG_FILE_LOCATION",
					RobobuggyConfigFile.LOG_FILE_LOCATION);

			logging = (boolean) obj.getOrDefault("LOGGING",
					RobobuggyConfigFile.LOGGING_DEFAULT);
			GUI_ON = (boolean) obj.getOrDefault("GUI", RobobuggyConfigFile.GUI_ON_DEFAULT);

			AUTONOMOUS_DEFAULT = (boolean) obj.getOrDefault("AUTONOMOUS",
					RobobuggyConfigFile.AUTONOMOUS_DEFAULT);
			DRIVE_DEFAULT = (boolean) obj.getOrDefault("DRIVE",
					RobobuggyConfigFile.DRIVE_DEFAULT);

			IMU_DEFAULT = (boolean) obj.getOrDefault("IMU", RobobuggyConfigFile.IMU_DEFAULT);
			GPS_DEFAULT = (boolean) obj.getOrDefault("GPS", RobobuggyConfigFile.GPS_DEFAULT);
			ENCODER_DEFAULT = (boolean) obj.getOrDefault("ENCODER",
					RobobuggyConfigFile.ENCODER_DEFAULT);
			VISION_SYSTEM_DEFAULT = (boolean) obj.getOrDefault("VISION",
					RobobuggyConfigFile.VISION_SYSTEM_DEFAULT);
			BRAKES_PER_FULL_PRESSURE = (byte) obj.getOrDefault(
					"BRAKE_PER_FULL_PRESSURE", RobobuggyConfigFile.BRAKES_PER_FULL_PRESSURE);
		} catch (FileNotFoundException e) {
			System.out.println("Failed to read file: " + filename);
		} catch (IOException e) {
			System.out.println("Failed to read file: " + filename);
		} catch (ParseException e) {
			System.out.println("Incorrectly formated JSON for file: "
					+ filename);
		}
		
		

	}
	//includes all of the jni libraries that we need to be able to use all of our libraries
	public static boolean setupJNI() throws NoSuchFieldException, SecurityException, IllegalArgumentException, IllegalAccessException{
	       String pathToAdd =  "library";

	       
	       
	       final Field usrPathsField = ClassLoader.class.getDeclaredField("usr_paths");
	       usrPathsField.setAccessible(true);

	       //get array of paths
	       final String[] paths = (String[])usrPathsField.get(null);

	       //check if the path to add is already present
	       for(String path : paths) {
	           if(path.equals(pathToAdd)) {
	               return true;
	           }
	       }

	       //add the new path
	       final String[] newPaths = Arrays.copyOf(paths, paths.length + 1);
	       newPaths[newPaths.length-1] = pathToAdd;
	       usrPathsField.set(null, newPaths);
	       
	       return true;
	}

}
