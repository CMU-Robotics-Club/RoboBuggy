package com.roboclub.robobuggy.simulation;

import java.io.FileReader;
import java.io.IOException;
import java.util.Date;

import com.orsoncharts.util.json.JSONArray;
import com.orsoncharts.util.json.JSONObject;
import com.orsoncharts.util.json.parser.JSONParser;
import com.orsoncharts.util.json.parser.ParseException;
import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.main.RobobuggyLogicException;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage.LoggingMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.utilities.RobobuggyDateFormatter;

public class SensorPlayer implements Runnable {

	private String path;
	
	private Publisher imuPub;
	private Publisher gpsPub;
	private Publisher encoderPub;
	private Publisher brakePub;
	private Publisher steeringPub;
	private Publisher loggingButtonPub;


	public SensorPlayer(String filePath) {
	
		imuPub = new Publisher(SensorChannel.IMU.toString());
		gpsPub = new Publisher(SensorChannel.GPS.toString());
		encoderPub = new Publisher(SensorChannel.ENCODER.toString());
		brakePub = new Publisher(SensorChannel.BRAKE.toString());
		steeringPub = new Publisher(SensorChannel.STEERING.toString());
		loggingButtonPub = new Publisher(SensorChannel.GUI_LOGGING_BUTTON.toString());
		
		System.out.println("initializing the SensorPlayer");
	}
	
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		
		System.out.println("Starting to read from file!");
		try {
			JSONParser parser = new JSONParser();
			FileReader reader = new FileReader(path);
			JSONObject completeLogFile = (JSONObject) parser.parse(reader); //may take a while, currently working on a solution to read line by line
			
			//get info from the header file
			Date loggingDate = new Date();
			
			long prevTimeInMillis = loggingDate.getTime();
			
			JSONArray sensorDataArray = (JSONArray) completeLogFile.get("sensor_data");
			for(Object senObj : sensorDataArray) {
				
				JSONObject sensor = (JSONObject)senObj;
				
				Date sensorTimestamp = (Date) sensor.get("timestamp");
				long currentSensorTimeInMillis = sensorTimestamp.getTime();
				long sleepTime = currentSensorTimeInMillis - prevTimeInMillis;
				if(sleepTime > 0) {
					Thread.sleep(sleepTime);
				}
				prevTimeInMillis = currentSensorTimeInMillis;
				
				JSONObject sensorData = (JSONObject) sensor.get("data");
				String sensorName = (String) sensorData.get("name");
				
				JSONObject sensorParams = (JSONObject) sensorData.get("params");
				
				switch(sensorName) {
				
					case "IMU":
						float yaw = (float) sensorParams.get("yaw");
						float pitch = (float) sensorParams.get("pitch");
						float roll = (float) sensorParams.get("roll");
						
						imuPub.publish(new ImuMeasurement(yaw, pitch, roll));
						
						break;
						
					case "GPS":
						
						double latitude = (double) sensorParams.get("latitude");
						double longitude = (double) sensorParams.get("longitude");
						String latDir = (String) sensorParams.get("lat_direction");
						String longDir = (String) sensorParams.get("long_direction");
						boolean north = latDir.equals("N");
						boolean west = longDir.equals("W");

						String gpsTimestampString = (String) sensorData.get("timestamp");
						Date gpsTimestamp = RobobuggyDateFormatter.formatRobobuggyDate(gpsTimestampString);
						int qualityValue = Integer.valueOf((String) sensorParams.get("gps_quality"));
						int numSatellites = Integer.valueOf((String) sensorParams.get("num_satellites"));
						double hdop = (double) sensorParams.get("HDOP");
						float antennaAlt = (float) sensorParams.get("antenna_altitude");
						
						gpsPub.publish(new GpsMeasurement(gpsTimestamp, latitude, north, longitude, west, qualityValue, numSatellites, hdop, antennaAlt));
						
						break;
						
					case "logging button":
						
						String loggingStatus = (String) sensorParams.get("logging_status");
						GuiLoggingButtonMessage.LoggingMessage loggingMessage = LoggingMessage.STOP;
						
						switch (loggingStatus) {
						
							case "start":
								loggingMessage = LoggingMessage.START;
								break;
								
							case "stop":
								loggingMessage = LoggingMessage.STOP;
								break;
								
							default:
								new RobobuggyLogicException("Unknown status in the log!", MessageLevel.EXCEPTION);
								break;
						
						}
						
						loggingButtonPub.publish(new GuiLoggingButtonMessage(loggingMessage));
						
						break;
						
					case "Steering":
						
						float steeringAngle = (float) sensorParams.get("angle");
						steeringPub.publish(new SteeringMeasurement((int) steeringAngle));
						
						break;
						
					case "Encoder":
						
						double dataword = (double) sensorParams.get("dataword");
						double distance = (double) sensorParams.get("distance");
						double velocity = (double) sensorParams.get("velocity");
						double accel = (double) sensorParams.get("acceleration");
						
						String timestampString = (String) sensorData.get("timestamp");
						Date timestamp = RobobuggyDateFormatter.formatRobobuggyDate(timestampString);
						
						encoderPub.publish(new EncoderMeasurement(timestamp, dataword, distance, velocity, accel));
						
						break;
						
					case "Brake":
						//we don't log brakes yet!
						//https://www.youtube.com/watch?v=UKDDu_0NsEk
						break;
						
					default:
						new RobobuggyLogicException("Found an unsupported sensor in the logs!", MessageLevel.WARNING);
						break;
				
				}
				
				
			}
			
		}
		catch(IOException e) {
			new RobobuggyLogicException("Error while trying to read from the playback file!", MessageLevel.EXCEPTION);
		} catch (InterruptedException e) {
			new RobobuggyLogicException("Something went wrong while waiting...", MessageLevel.WARNING);
		} catch (ParseException e) {
			new RobobuggyLogicException("Couldn't parse the file given. Make sure it's a proper JSON file", MessageLevel.EXCEPTION);
		}		
	}
}