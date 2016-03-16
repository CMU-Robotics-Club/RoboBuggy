package com.roboclub.robobuggy.main;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.UnsupportedEncodingException;

import com.google.gson.JsonObject;

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
	private static boolean imuEnabled = true;
	private static boolean gpsEnabled = true;
	private static boolean encoderEnabled = true;
	private static boolean visionSystemEnabled = true;

	//sensor com ports
	private static String comPortImu = "NOT_SET";

	private static String comPortGps = "NOT_SET";
	private static String comPortRBSM = "NOT_SET";
	private static String portVision ="NOT_SET";

	// iff false, connect to serial sensors 
	private static final boolean DATA_PLAY_BACK = true;

	private static String waypointSourceLogFile ="NOT_SET";
	private static String playBackSourceFile = "NOT_SET";
	
	//is where values from this file are saved so that they can be updated between runs
	private static String configFile ="config.config";
	
	/**
	 * creates a .config file that stores as json the current configuration settings 
	 * the file is saved based on the result of getCONFIG_FILE()
	 */
	public static void saveConfigFile(){
		JsonObject settings = new JsonObject();
		settings.addProperty("COM_PORT_IMU", getCOM_PORT_IMU());
		settings.addProperty("COM_PORT_RBSM", getCOM_PORT_RBSM());
		settings.addProperty("PORT_VISION", getPORT_VISION());
		settings.addProperty("COM_PORT_GPS", getCOM_PORT_GPS());
		settings.addProperty("WAYPOINT_SOURCE_LOG_FILE", getWAYPOINT_SOURCE_LOG_FILE());
		settings.addProperty("PLAY_BACK_SOURCE_FILE", getPLAY_BACK_SOURCE_FILE());
		try (FileWriter file = new FileWriter(getCONFIG_FILE())) {
			file.write(settings.toString());
		}
		//TODO add other settings as they are created 
		catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * Helper function which removes the first and last elements from a string
	 * @param input
	 * @return quote free string
	 */
	private static String removeQuotes(String input){
		//the string is too short
		if(input.length() < 2){
			return "";
		}
		return input.substring(1, input.length() -1);
	}
	
	/**
	 * Attempts to load values for configuration settings based on the configuration file
	 * If the configuration file cannot be read then the old values will be used 
	 */
	public static void loadConfigFile(){
		try {
			JsonObject configJson = Util.readJSONFile(getCONFIG_FILE());
			setCOM_PORT_IMU(removeQuotes(configJson.get("COM_PORT_IMU").toString()));
			setCOM_PORT_GPS(removeQuotes(configJson.get("COM_PORT_GPS").toString()));
			setCOM_PORT_RBSM(removeQuotes(configJson.get("COM_PORT_RBSM").toString()));
			setPORT_VISION(removeQuotes(configJson.get("PORT_VISION").toString()));
			setWAYPOINT_SOURCE_LOG_FILE(removeQuotes(configJson.get("WAYPOINT_SOURCE_LOG_FILE").toString()));
			setPLAY_BACK_SOURCE_FILE(removeQuotes(configJson.get("PLAY_BACK_SOURCE_FILE").toString()));

			
		} catch (UnsupportedEncodingException | FileNotFoundException e) {
			new RobobuggyLogicNotification("could not load Part of configFile"+e.toString(), RobobuggyMessageLevel.WARNING);
		}
	}

	/**
	 * evaluates to the current value of imuEnabled
	 * @return
	 */
	public static boolean isImuEnabled() {
		return imuEnabled;
	}

	/**
	 * sets imuEnabled to a new value
	 * @param imuEnabled
	 */
	public static void setImuEnabled(boolean imuEnabled) {
		RobobuggyConfigFile.imuEnabled = imuEnabled;
	}

	/**
	 * evaluates to the current value of dataPlayBack
	 * @return
	 */
	public static boolean isDataPlayBack() {
		return DATA_PLAY_BACK;
	}

	/**
	 * evaluates to the current value of comPortImu 
	 * @return
	 */
	public static String getCOM_PORT_IMU() {
		return comPortImu;
	}

	/**
	 * sets a new value to comPortImu
	 * @param cOM_PORT_IMU
	 */
	public static void setCOM_PORT_IMU(String cOM_PORT_IMU) {
		comPortImu = cOM_PORT_IMU;
	}

	/**
	 * evaluates to the current value of configFile
	 * @return
	 */
	public static String getCONFIG_FILE() {
		return configFile;
	}

	/**
	 * sets a new value to configFile
	 * @param cONFIG_FILE
	 */
	public static void setCONFIG_FILE(String cONFIG_FILE) {
		configFile = cONFIG_FILE;
	}

	/**
	 * evaluates to the current value of waypointSourceLogFile
	 * @return
	 */
	public static String getWAYPOINT_SOURCE_LOG_FILE() {
		return waypointSourceLogFile;
	}

	/**
	 * sets a new value to waypointSourceLogFile
	 * @param wAYPOINT_SOURCE_LOG_FILE
	 */
	public static void setWAYPOINT_SOURCE_LOG_FILE(
			String wAYPOINT_SOURCE_LOG_FILE) {
		waypointSourceLogFile = wAYPOINT_SOURCE_LOG_FILE;
	}

	/**
     * evaluates to the current value of playBackSourceFile
	 * @return
	 */
	public static String getPLAY_BACK_SOURCE_FILE() {
		return playBackSourceFile;
	}

	/**
	 * sets a new value to playBackSourceFile
	 * @param pLAY_BACK_SOURCE_FILE
	 */
	public static void setPLAY_BACK_SOURCE_FILE(String pLAY_BACK_SOURCE_FILE) {
		playBackSourceFile = pLAY_BACK_SOURCE_FILE;
	}

	/**
	 * evaluates to the currentValue of comPortGps
	 * @return
	 */
	public static String getCOM_PORT_GPS() {
		return comPortGps;
	}

	/**
	 * sets a  new value to comPortGps
	 * @param cOM_PORT_GPS
	 */
	public static void setCOM_PORT_GPS(String cOM_PORT_GPS) {
		comPortGps = cOM_PORT_GPS;
	}

	/**
	 * evaluates to the current value of gpsEnabled
	 * @return
	 */
	public static boolean isGps_enabled() {
		return gpsEnabled;
	}

	/**
	 * sets a new value to gpsEnabled
	 * @param gps_enabled
	 */
	public static void setGps_enabled(boolean gps_enabled) {
		RobobuggyConfigFile.gpsEnabled = gps_enabled;
	}

	/**
	 * evaluates to the current value of comPortRBSM
	 * @return
	 */
	public static String getCOM_PORT_RBSM() {
		return comPortRBSM;
	}

	/**
	 * sets a new value to comPortRBSM
	 * @param cOM_PORT_RBSM
	 */
	public static void setCOM_PORT_RBSM(String cOM_PORT_RBSM) {
		comPortRBSM = cOM_PORT_RBSM;
	}

	/**
	 * evaluates to the current value of encoderEnabled
	 * @return
	 */
	public static boolean isEncoder_enabled() {
		return encoderEnabled;
	}

	/**
	 * sets a new value to encoderEnabled
	 * @param encoder_enabled
	 */
	public static void setEncoder_enabled(boolean encoder_enabled) {
		RobobuggyConfigFile.encoderEnabled = encoder_enabled;
	}

	/**
	 * Evaluates to the current value of portVision
	 * @return
	 */
	public static String getPORT_VISION() {
		return portVision;
	}

	/**
	 * sets a new value to portVision
	 * @param pORT_VISION
	 */
	public static void setPORT_VISION(String pORT_VISION) {
		portVision = pORT_VISION;
	}

	/**
	 * evaluates to the current value of visionSystemEnabled
	 * @return
	 */
	public static boolean isVISION_SYSTEM_ENABLED() {
		return visionSystemEnabled;
	}

	/**
	 * sets a new value for visionSystemEnabled
	 * @param visionSystemEnabled
	 */
	public static void setVisionSystemEnabled(boolean visionSystemEnabled) {
		RobobuggyConfigFile.visionSystemEnabled = visionSystemEnabled;
	}
	
	
}
