package com.roboclub.robobuggy.main;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import com.orsoncharts.util.json.JSONObject;
import com.orsoncharts.util.json.parser.JSONParser;
import com.orsoncharts.util.json.parser.ParseException;

// where all system configuration values should be placed
// add jason parser for config

public class config {
	// port index of the front camera
	public static int FRONT_CAM_INDEX = 2;

	// port index of the rear (pushbar camera)
	public static int REAR_CAM_INDEX = 0;

	// location of the executable that should be run for the camera sub system
	public static String VISION_SYSTEM_EXECUTABLE_LOCATION = "C:\\Users\\Robot\\Documents\\GitHub\\RoboBuggy\\surface_src\\VisionSystem\\Debug\\VisionSystem.exe";
	// "C:\\Users\\abc\\buggy-log\\VisionSystem.exe";

	public static String LOG_FILE_LOCATION = "C:\\Users\\abc\\buggy-log";

	// default logging state, should the buggy start logging as soon as this
	// program is started
	public static boolean LOGGING_DEFAULT = true;

	// default running state, should the buggy program start running as soon as
	// this program is started
	public static boolean ACTIVE_DEFAULT = false;

	public static boolean AUTONOMUS_DEFAULT = false;

	// sensor default settings (true for on false for off)

	public static boolean DRIVE_DEFAULT = true;
	public static boolean IMU_DEFAULT = true;
	public static boolean GPS_DEFAULT = true;
	public static boolean ENCODER_DEFAULT = true;
	public static boolean VISION_SYSTEM_DEFAULT = true;

	// number of times that we will allow for the brakes to be deployed and
	// still have the buggy run
	public static byte BRAKES_PER_FULL_PRESSURE = 4;

	public static boolean GUI_ON_DEFAULT = true;
	public static boolean DATA_PLAY_BACK_DEFAULT = false;// if false then try to

	// current status values
	public static boolean GUI_ON;
	public static boolean active;
	public static boolean logging;

	public static String clEyeDllPath = "C://CL-Eye Platform SDK//Bin//CLEyeMulticam.dll";

	// internal reference of this config so that it can fit the factory pattern
	private static config instance;

	public static config getInstance() {
		if (instance == null) {
			instance = new config();
			// sets defaults
			logging = config.LOGGING_DEFAULT;
			GUI_ON = GUI_ON_DEFAULT;
			active = ACTIVE_DEFAULT;
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
					config.FRONT_CAM_INDEX);
			REAR_CAM_INDEX = (int) obj.getOrDefault("BACK_CAM_INDEX",
					config.REAR_CAM_INDEX);
			VISION_SYSTEM_EXECUTABLE_LOCATION = (String) obj.getOrDefault(
					"VISION_SYSTEM_EXECUTABLE_LOCATION",
					config.VISION_SYSTEM_EXECUTABLE_LOCATION);
			LOG_FILE_LOCATION = (String) obj.getOrDefault("LOG_FILE_LOCATION",
					config.LOG_FILE_LOCATION);

			logging = (boolean) obj.getOrDefault("LOGGING",
					config.LOGGING_DEFAULT);
			GUI_ON = (boolean) obj.getOrDefault("GUI", config.GUI_ON_DEFAULT);

			AUTONOMUS_DEFAULT = (boolean) obj.getOrDefault("AUTONOMOUS",
					config.AUTONOMUS_DEFAULT);
			DRIVE_DEFAULT = (boolean) obj.getOrDefault("DRIVE",
					config.DRIVE_DEFAULT);

			IMU_DEFAULT = (boolean) obj.getOrDefault("IMU", config.IMU_DEFAULT);
			GPS_DEFAULT = (boolean) obj.getOrDefault("GPS", config.GPS_DEFAULT);
			ENCODER_DEFAULT = (boolean) obj.getOrDefault("ENCODER",
					config.ENCODER_DEFAULT);
			VISION_SYSTEM_DEFAULT = (boolean) obj.getOrDefault("VISION",
					config.VISION_SYSTEM_DEFAULT);
			BRAKES_PER_FULL_PRESSURE = (byte) obj.getOrDefault(
					"BRAKE_PER_FULL_PRESSURE", config.BRAKES_PER_FULL_PRESSURE);
		} catch (FileNotFoundException e) {
			System.out.println("Failed to read file: " + filename);
		} catch (IOException e) {
			System.out.println("Failed to read file: " + filename);
		} catch (ParseException e) {
			System.out.println("Incorrectly formated JSON for file: "
					+ filename);
		}
	}

}
