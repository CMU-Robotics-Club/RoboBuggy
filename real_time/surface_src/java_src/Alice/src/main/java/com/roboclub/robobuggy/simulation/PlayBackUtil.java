package com.roboclub.robobuggy.simulation;

import com.google.gson.JsonObject;

/**
 * utilities for playback
 */
public class PlayBackUtil {
	  private static final String METADATA_NAME = "Robobuggy Data Logs";
	  private static final String METADATA_SCHEMA_VERSION = "1.1";
	  private static final String METADATA_HIGHLEVEL_SW_VERSION = "1.0.0";


    /**
     * validates the log file metadata
     * @param logFile the log file to validate
     * @return whether or not the log file is valid
     */
    public static boolean validateLogFileMetadata(JsonObject logFile) {
    	if(logFile == null){
    		return false;
    	}
    	
        if (!logFile.get("name").getAsString().equals(METADATA_NAME)) {
            return false;
        }
        if (!logFile.get("schema_version").getAsString().equals(METADATA_SCHEMA_VERSION)) {
            return false;
        }
        if (!logFile.get("software_version").getAsString().equals(METADATA_HIGHLEVEL_SW_VERSION)) {
            return false;
        }

        return true;
    }
	
}
