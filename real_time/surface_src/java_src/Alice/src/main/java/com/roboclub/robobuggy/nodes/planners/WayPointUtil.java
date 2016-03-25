package com.roboclub.robobuggy.nodes.planners;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.simulation.PlayBackUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Date;
import java.util.Scanner;

/**
 * Parses a log file and creates a set of waypoints from the relevant data
 * @author Trevor Decker
 *
 */
public class WayPointUtil {


	/**
	 * @param filename log file
	 * @return arraylist of waypoints
	 * @throws FileNotFoundException if we couldn't find the log file
	 */
	public static ArrayList<Message> createWayPointsFromWaypointList(String filename) throws FileNotFoundException {
		ArrayList<Message> waypoints = new ArrayList<>();

		File waypointFile = new File(filename);
		Gson translator = new GsonBuilder().serializeSpecialFloatingPointValues().create();
		Scanner fileReader = new Scanner(new FileInputStream(waypointFile), "UTF-8");

		while (fileReader.hasNextLine()) {
			String nextline = fileReader.nextLine();
			if (nextline.equals("")) {
				break;
			}
			waypoints.add(translator.fromJson(nextline, GpsMeasurement.class));
		}

		fileReader.close();

		return waypoints;
	}

	/**
	 * @param filename log file with odom localizations
	 * @return waypoint list
	 * @throws FileNotFoundException if we couldn't find the log file
	 * @throws UnsupportedEncodingException if you're stupid and not using a utf encoded file :)
	 */
	public static ArrayList<GpsMeasurement> createWaypointsFromOdomLocalizerLog(String filename)
										throws FileNotFoundException, UnsupportedEncodingException {
		File odomLog = new File(filename);
		Gson translator = new GsonBuilder().create();
		ArrayList<GpsMeasurement> waypoints = new ArrayList<>();

		InputStreamReader stream = new InputStreamReader(new FileInputStream(odomLog), "UTF-8");
		JsonObject logFile = translator.fromJson(stream, JsonObject.class);

		if(PlayBackUtil.validateLogFileMetadata(logFile)) {
			JsonArray sensorDataArray = logFile.getAsJsonArray("sensor_data");
			for (JsonElement sensorAsJElement : sensorDataArray) {
				JsonObject sensorDataJson = sensorAsJElement.getAsJsonObject();
				String versionId = sensorDataJson.get("VERSION_ID").getAsString();

				Message transmit;

				switch (versionId) {
					case GPSPoseMessage.VERSION_ID:
						transmit = translator.fromJson(sensorDataJson, GPSPoseMessage.class);
						GPSPoseMessage pose = (GPSPoseMessage) transmit;
						GpsMeasurement waypoint = new GpsMeasurement(new Date(), pose.getLatitude(),
								true, pose.getLongitude(), true, 0, 0, 0, 0, 0, 0);
						waypoints.add(waypoint);
						break;
					default:
						break;
				}
			}
		}

		return waypoints;
	}

	/**
	 * also places a log file just filled with waypoints
	 * @param folder the folder name
	 * @param filename the log file name
	 * @return waypoint list
	 * @throws IOException we couldn't find the file or folder
	 */
	public static ArrayList createWayPointsFromLog(String folder, String filename) throws IOException {
			   
				ArrayList messages = new ArrayList();
				File outputFile = new File(folder + "/waypoints.txt");
				if (!outputFile.createNewFile()) {
					new RobobuggyLogicNotification("couldn't create file", RobobuggyMessageLevel.EXCEPTION);
//					return null;
				}
				OutputStreamWriter writer = new OutputStreamWriter(new FileOutputStream(outputFile), "UTF-8");

				Gson translator = new GsonBuilder().serializeSpecialFloatingPointValues().create();
	            InputStreamReader fileReader = new InputStreamReader(new FileInputStream(new File(folder + "/" + filename)), "UTF-8");
	            JsonObject logFile = translator.fromJson(fileReader, JsonObject.class);

	            if(!PlayBackUtil.validateLogFileMetadata(logFile)) {
	                new RobobuggyLogicNotification("Log file doesn't have the proper header metadata!", RobobuggyMessageLevel.EXCEPTION);
					writer.close();
					fileReader.close();
	                return messages;
	            }


	            JsonArray sensorDataArray = logFile.getAsJsonArray("sensor_data");
	            for (JsonElement sensorAsJElement: sensorDataArray) {
	                if(sensorAsJElement.isJsonObject()){
	                    JsonObject sensorDataJson = sensorAsJElement.getAsJsonObject();
	                    String versionID = sensorDataJson.get("VERSION_ID").getAsString();


	                    switch (versionID) {
	                        case GpsMeasurement.VERSION_ID:
	                        	Message transmitMessage = translator.fromJson(sensorDataJson, GpsMeasurement.class);
	                            messages.add(transmitMessage);
								writer.write(sensorDataJson.toString() + "\n");
	                            break;
							default:
								break;
	                    }
	                   }
	                }//for loop

			writer.flush();
		writer.close();
		fileReader.close();
	            return messages;
	}
	

}
