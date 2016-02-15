package com.roboclub.robobuggy.nodes.planners;

import java.io.*;
import java.lang.reflect.Array;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Scanner;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.BaseMessage;
import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.FingerPrintMessage;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.RemoteWheelAngleRequest;
import com.roboclub.robobuggy.messages.ResetMessage;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.messages.WheelAngleCommandMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.simulation.PlayBackUtil;

/**
 * Pares a log file and 
 * @author Trevor Decker
 *
 */
public class WayPointUtil {


	public static ArrayList<Message> CreateWayPointsFromWaypointList(String filename) throws FileNotFoundException {
		ArrayList<Message> waypoints = new ArrayList<>();

		File waypointFile = new File(filename);
		Gson translator = new GsonBuilder().serializeSpecialFloatingPointValues().create();
		Scanner fileReader = new Scanner(new FileInputStream(waypointFile));

		while (fileReader.hasNextLine()) {
			String nextline = fileReader.nextLine();
			if (nextline.equals("")) {
				break;
			}
			waypoints.add(translator.fromJson(nextline, GpsMeasurement.class));
		}

		return waypoints;
	}
	
	public static ArrayList CreateWayPointsFromLog(String folder, String filename) throws IOException {
			   
				ArrayList messages = new ArrayList();
				File outputFile = new File(folder + "/waypoints.txt");
				outputFile.createNewFile();
				FileWriter writer = new FileWriter(outputFile);

				Gson translator = new GsonBuilder().serializeSpecialFloatingPointValues().create();
	            InputStreamReader fileReader = new InputStreamReader(new FileInputStream(new File(folder + "/" + filename)), "UTF-8");
	            JsonObject logFile = translator.fromJson(fileReader, JsonObject.class);

	            if(!PlayBackUtil.validateLogFileMetadata(logFile)) {
	                new RobobuggyLogicNotification("Log file doesn't have the proper header metadata!", RobobuggyMessageLevel.EXCEPTION);
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
	                    };
	                   }
	                }//for loop

			writer.flush();
		writer.close();
	            return messages;
	}

		 
}
