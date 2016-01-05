package com.roboclub.robobuggy.simulation;

import java.io.File;
import java.io.FileNotFoundException;
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
import com.roboclub.robobuggy.ros.NodeChannel;
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
	
		imuPub = new Publisher(NodeChannel.IMU.getMsgPath());
		gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
		encoderPub = new Publisher(NodeChannel.ENCODER.getMsgPath());
		brakePub = new Publisher(NodeChannel.BRAKE.getMsgPath());
		steeringPub = new Publisher(NodeChannel.STEERING.getMsgPath());
		loggingButtonPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
		
		System.out.println("initializing the SensorPlayer");
		
		path = filePath;
		File f = new File(path);
		if(!f.exists()) {
			new RobobuggyLogicException("File doesn't exist!", MessageLevel.EXCEPTION);
		}
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
			long sensorStartTimeInMilis = 0;
			for(Object senObj : sensorDataArray) {
				
				JSONObject sensor = (JSONObject)senObj;
				
				Date sensorTimestamp = RobobuggyDateFormatter.formatRobobuggyDate((String) sensor.get("timestamp"));
				long currentSensorTimeInMillis = sensorTimestamp.getTime();
				long currentTime = loggingDate.getTime();

				//grab the first sensors time for reference
				if(sensorStartTimeInMilis == 0){
					sensorStartTimeInMilis = currentSensorTimeInMillis;
				}
				long sensorTime_fromStart = currentSensorTimeInMillis -sensorStartTimeInMilis; 
				long realTime_fromStart = currentTime - prevTimeInMillis;				
				long PLAY_BACK_SPEED = 100;
				long sleepTime = PLAY_BACK_SPEED*realTime_fromStart - sensorTime_fromStart;
				System.out.println("sleepTime:"+sleepTime);
				if(sleepTime < 0 ){ 
					//TODO change back to sleepTime
					Thread.sleep(-sleepTime/10000);
//					Thread.sleep(500);
				}
				//prevTimeInMillis = currentSensorTimeInMillis;
			
				String sensorName = (String) sensor.get("name");
				
				JSONObject sensorParams = (JSONObject) sensor.get("params");
				
				switch(sensorName) {
				
					case "IMU":
						double yaw = (double) sensorParams.get("yaw");
						double pitch = (double) sensorParams.get("pitch");
						double roll = (double) sensorParams.get("roll");
						
						imuPub.publish(new ImuMeasurement(yaw, pitch, roll));
						
						break;
						
					case "GPS":
						
						double latitude = (double) sensorParams.get("latitude");
						double longitude = (double) sensorParams.get("longitude");
						String latDir = (String) sensorParams.get("lat_direction");
						String longDir = (String) sensorParams.get("long_direction");
						boolean north = latDir.equals("N");
						boolean west = longDir.equals("W");

						String gpsTimestampString = (String) sensor.get("timestamp");
						Date gpsTimestamp = RobobuggyDateFormatter.formatRobobuggyDate(gpsTimestampString);
						int qualityValue = Integer.valueOf((String) sensorParams.get("gps_quality"));
						int numSatellites = Integer.valueOf((String) sensorParams.get("num_satellites"));
						double hdop = (double) sensorParams.get("HDOP");
						double antennaAlt = (double) sensorParams.get("antenna_altitude");
						double rawLat = (double) sensorParams.get("raw_gps_lat");
						double rawLon = (double) sensorParams.get("raw_gps_lon");
						
						gpsPub.publish(new GpsMeasurement(gpsTimestamp, latitude, north, longitude, west, qualityValue, numSatellites, hdop, antennaAlt, rawLat, rawLon));
						
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
						
						double steeringAngle = (double) sensorParams.get("angle");
						steeringPub.publish(new SteeringMeasurement((int) steeringAngle));
						
						break;
						
					case "Encoder":
						
						double dataword = (double) sensorParams.get("dataword");
						double distance = (double) sensorParams.get("distance");
						double velocity = sensorParams.get("velocity") != null ? (double) sensorParams.get("velocity") : 0;
						Double accel = sensorParams.get("acceleration") != null ? (double) sensorParams.get("acceleration") : 0;
						
						String timestampString = (String) sensor.get("timestamp");
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
				
				System.out.println("sent out data from " + sensorName);
				
				
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