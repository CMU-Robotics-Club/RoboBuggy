package com.roboclub.robobuggy.main;

import com.google.gson.JsonObject;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.io.Writer;

// add JSON parser for RobobuggyConfigFile

/**
 * Class used to store the system configuration values
 */
public final class RobobuggyConfigFile {

    //Library info
    public static final String ALICE_LIBRARY_VERSION = "1.0.0";
    public static final String RBSM_HEADER_FILE_LOCATION = "../../../rbsm_config.txt";

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
    public static final int RBSM_COMMAND_PERIOD = 50;

    // sensor default settings (true for on false for off
    private static boolean imuEnabled = true;
    private static boolean gpsEnabled = true;
    private static boolean encoderEnabled = true;
    private static boolean visionSystemEnabled = true;

    //system settings
    private static double playBackSpeed = 1.0;

    //sensor com ports
    private static String comPortImu = "NOT_SET";
    private static String comPortGps = "NOT_SET";
    private static String comPortRBSM = "NOT_SET";
    private static String portVision = "NOT_SET";


    // iff false, connect to serial sensors
    private static final boolean DATA_PLAY_BACK = false;

    private static String waypointSourceLogFile = "NOT_SET";
    private static String playBackSourceFile = "NOT_SET";

    //is where values from this file are saved so that they can be updated between runs
    private static String configFile = "config.config";

    /**
     * creates a .config file that stores as json the current configuration settings
     * the file is saved based on the result of getCONFIG_FILE()
     */
    public static void saveConfigFile() {
        JsonObject settings = new JsonObject();
        settings.addProperty("COM_PORT_IMU", getComPortImu());
        settings.addProperty("COM_PORT_RBSM", getComPortRBSM());
        settings.addProperty("PORT_VISION", getPortVision());
        settings.addProperty("COM_PORT_GPS", getComPortGPS());
        settings.addProperty("WAYPOINT_SOURCE_LOG_FILE", getWaypointSourceLogFile());
        settings.addProperty("PLAY_BACK_SOURCE_FILE", getPlayBackSourceFile());
        try {
            File file = new File(getConfigFile());
            Writer w = new OutputStreamWriter(new FileOutputStream(file), "UTF-8");
            PrintWriter pw = new PrintWriter(w);
            pw.println(settings.toString());
            pw.close();
        }
        //TODO add other settings as they are created
        catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    /**
     * Attempts to load values for configuration settings based on the configuration file
     * If the configuration file cannot be read then the old values will be used
     */
    public static void loadConfigFile() {
        try {
            JsonObject configJson = Util.readJSONFile(getConfigFile());
            setComPortImu(configJson.get("COM_PORT_IMU").getAsString());
            setComPortGps(configJson.get("COM_PORT_GPS").getAsString());
            setComPortRBSM(configJson.get("COM_PORT_RBSM").getAsString());
            setPortVision(configJson.get("PORT_VISION").getAsString());
            setWayPointSourceLogFile(configJson.get("WAYPOINT_SOURCE_LOG_FILE").getAsString());
            setPlayBackSourceFile(configJson.get("PLAY_BACK_SOURCE_FILE").getAsString());


        } catch (UnsupportedEncodingException | FileNotFoundException e) {
            new RobobuggyLogicNotification("could not load Part of configFile" + e.toString(), RobobuggyMessageLevel.WARNING);
        }
    }

    /**
     * evaluates to the current value of imuEnabled
     *
     * @return current value of imuEnabled
     */
    public static boolean isImuEnabled() {
        return imuEnabled;
    }

    /**
     * sets imuEnabled to a new value
     *
     * @param imuEnabled boolean newValue
     */
    public static void setImuEnabled(boolean imuEnabled) {
        RobobuggyConfigFile.imuEnabled = imuEnabled;
    }

    /**
     * evaluates to the current value of dataPlayBack
     *
     * @return current value of dataPlayBack
     */
    public static boolean isDataPlayBack() {
        return DATA_PLAY_BACK;
    }

    /**
     * evaluates to the current value of comPortImu
     *
     * @return current value of comPortImu
     */
    public static String getComPortImu() {
        return comPortImu;
    }

    /**
     * sets a new value to comPortImu
     *
     * @param comPortImu boolean newValue
     */
    public static void setComPortImu(String comPortImu) {
        RobobuggyConfigFile.comPortImu = comPortImu;
    }

    /**
     * evaluates to the current value of configFile
     *
     * @return current value of configFile
     */
    public static String getConfigFile() {
        return configFile;
    }

    /**
     * sets a new value to configFile
     *
     * @param configFile new value
     */
    public static void setConfigFile(String configFile) {
        RobobuggyConfigFile.configFile = configFile;
    }

    /**
     * evaluates to the current value of waypointSourceLogFile
     *
     * @return current value of waypointSourceLogFile
     */
    public static String getWaypointSourceLogFile() {
        return waypointSourceLogFile;
    }

    /**
     * sets a new value to waypointSourceLogFile
     *
     * @param waypointSourceLogFile new String
     */
    public static void setWayPointSourceLogFile(String waypointSourceLogFile) {
        RobobuggyConfigFile.waypointSourceLogFile = waypointSourceLogFile;
    }

    /**
     * evaluates to the current value of playBackSourceFile
     *
     * @return current value of playBackSourceFile
     */
    public static String getPlayBackSourceFile() {
        return playBackSourceFile;
    }

    /**
     * sets a new value to playBackSourceFile
     *
     * @param playBackSourceFile new String
     */
    public static void setPlayBackSourceFile(String playBackSourceFile) {
        RobobuggyConfigFile.playBackSourceFile = playBackSourceFile;
    }

    /**
     * evaluates to the currentValue of comPortGps
     *
     * @return currentValue of comPortGps
     */
    public static String getComPortGPS() {
        return comPortGps;
    }

    /**
     * sets a  new value to comPortGps
     *
     * @param comPortGPS String new value
     */
    public static void setComPortGps(String comPortGPS) {
        comPortGps = comPortGPS;
    }

    /**
     * evaluates to the current value of gpsEnabled
     *
     * @return current value of gpsEnabled
     */
    public static boolean isGpsEnabled() {
        return gpsEnabled;
    }

    /**
     * sets a new value to gpsEnabled
     *
     * @param gpsEnabled boolean new Value
     */
    public static void setGpsEnabled(boolean gpsEnabled) {
        RobobuggyConfigFile.gpsEnabled = gpsEnabled;
    }

    /**
     * evaluates to the current value of comPortRBSM
     *
     * @return current value of comPortRBSM
     */
    public static String getComPortRBSM() {
        return comPortRBSM;
    }

    /**
     * sets a new value to comPortRBSM
     *
     * @param comPortRBSM String new value
     */
    public static void setComPortRBSM(String comPortRBSM) {
        RobobuggyConfigFile.comPortRBSM = comPortRBSM;
    }

    /**
     * evaluates to the current value of encoderEnabled
     *
     * @return current value of encoderEnabled
     */
    public static boolean isEncoderEnabled() {
        return encoderEnabled;
    }

    /**
     * sets a new value to encoderEnabled
     *
     * @param encoderEnabled boolean new value
     */
    public static void setEncoderEnabled(boolean encoderEnabled) {
        RobobuggyConfigFile.encoderEnabled = encoderEnabled;
    }

    /**
     * Evaluates to the current value of portVision
     *
     * @return current value of portVision
     */
    public static String getPortVision() {
        return portVision;
    }

    /**
     * sets a new value to portVision
     *
     * @param portVision new String
     */
    public static void setPortVision(String portVision) {
        RobobuggyConfigFile.portVision = portVision;
    }

    /**
     * evaluates to the current value of visionSystemEnabled
     *
     * @return current value of visionSystemEnabled
     */
    public static boolean isVisionSystemEnabled() {
        return visionSystemEnabled;
    }

    /**
     * sets a new value for visionSystemEnabled
     *
     * @param visionSystemEnabled boolean new value
     */
    public static void setVisionSystemEnabled(boolean visionSystemEnabled) {
        RobobuggyConfigFile.visionSystemEnabled = visionSystemEnabled;
    }

    /**
     * @return the playBackSpeed
     */
    public static double getPlayBackSpeed() {
        return playBackSpeed;
    }

    /**
     * @param playBackSpeed the playBackSpeed to set
     */
    public static void setPlayBackSpeed(double playBackSpeed) {
        RobobuggyConfigFile.playBackSpeed = playBackSpeed;
    }

}
