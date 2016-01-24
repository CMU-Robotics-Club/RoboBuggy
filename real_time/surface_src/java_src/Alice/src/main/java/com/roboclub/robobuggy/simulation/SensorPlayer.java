package com.roboclub.robobuggy.simulation;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Date;

import com.orsoncharts.util.json.JSONArray;
import com.orsoncharts.util.json.JSONObject;
import com.orsoncharts.util.json.parser.JSONParser;
import com.orsoncharts.util.json.parser.ParseException;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage.LoggingMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.utilities.RobobuggyDateFormatter;

/**
 * Class used for playing back old log files. It does this by reading BuggyROS 
 * messages stored in the log file and reinjecting them into the BuggyROS network.
 */
public class SensorPlayer implements Runnable {

	private String path;
	
	private Publisher imuPub;
	private Publisher gpsPub;
	private Publisher encoderPub;
	//private Publisher brakePub; TODO: future expansion 
	private Publisher steeringPub;
	private Publisher loggingButtonPub;


	/**
	 * Construct a new {@link SensorPlayer} object
	 * @param filePath {@link String} of the name and location of the log file
	 */
	public SensorPlayer(String filePath) {
	
		imuPub = new Publisher(NodeChannel.IMU.getMsgPath());
		gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
		encoderPub = new Publisher(NodeChannel.ENCODER.getMsgPath());
	//	brakePub = new Publisher(NodeChannel.BRAKE.getMsgPath()); TODO: future expansion 
		steeringPub = new Publisher(NodeChannel.STEERING.getMsgPath());
		loggingButtonPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
		
		System.out.println("initializing the SensorPlayer");
		
		path = filePath;
		File f = new File(path);
		if(!f.exists()) {
			new RobobuggyLogicNotification("File doesn't exist!", RobobuggyMessageLevel.EXCEPTION);
		}
	}
	
	
	@Override
	public void run() {
		// TODO Auto-generated method stub
		
		System.out.println("Starting to read from file!");
		try {
			JSONParser parser = new JSONParser();
			InputStreamReader reader = new InputStreamReader(new FileInputStream(path),"UTF-8");	
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

				long sensorTimeFromStart = currentSensorTimeInMillis -sensorStartTimeInMilis; 
				long realTimeFromStart = currentTime - prevTimeInMillis;				
				long playbackSpeed = 100;
				long sleepTime = playbackSpeed*realTimeFromStart - sensorTimeFromStart;
				new RobobuggyLogicNotification("sleepingTime:"+sleepTime, RobobuggyMessageLevel.NOTE);
				if(sleepTime < 0 && false){ 
					//TODO change back to sleepTime
					Thread.sleep(-sleepTime/1000000);
//					Thread.sleep(500);
				}
				String sensorName = (String) sensor.get("name");
				if(sensorName == null){
					new RobobuggyLogicNotification("sensor name is not in this lines log line, this log cannot be repaid",
							RobobuggyMessageLevel.EXCEPTION);
				}else{
					JSONObject sensorParams = (JSONObject) sensor.get("params");
					System.out.println(sensorName);
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
							int qualityValue = Integer.parseInt((String) sensorParams.get("gps_quality"));
							int numSatellites = Integer.parseInt((String) sensorParams.get("num_satellites"));
							double hdop = (double) sensorParams.get("HDOP");
							double antennaAlt = (double) sensorParams.get("antenna_altitude");
							double rawLat = (double) sensorParams.get("raw_gps_lat");
							double rawLon = (double) sensorParams.get("raw_gps_lon");		
							gpsPub.publish(new GpsMeasurement(gpsTimestamp, latitude, north, longitude, west,
									qualityValue, numSatellites, hdop, antennaAlt, rawLat, rawLon));
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
									new RobobuggyLogicNotification("Unknown status in the log!",
											RobobuggyMessageLevel.EXCEPTION);
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
							double velocity = 0;
							if(sensorParams.get("velocity") != null)
								velocity = (double) sensorParams.get("velocity");
							double accel = 0;
							if(sensorParams.get("acceleration") != null)
								accel = (double) sensorParams.get("acceleration");
						
							String timestampString = (String) sensor.get("timestamp");
							Date timestamp = RobobuggyDateFormatter.formatRobobuggyDate(timestampString);
						
							encoderPub.publish(new EncoderMeasurement(timestamp, dataword, distance, velocity, accel));
							break;
						
						case "Brake":
							//we don't log brakes yet!
							//https://www.youtube.com/watch?v=UKDDu_0NsEk
							break;
						
						default:
							//new RobobuggyLogicException("Found an unsupported sensor in the logs!",
							//RobobuggyMessageLevel.WARNING);
							break;
				
				}
								
				
			}
			}
			
		}
		catch(IOException e) {
			new RobobuggyLogicNotification("Error while trying to read from the playback file!", RobobuggyMessageLevel.EXCEPTION);
		} catch (InterruptedException e) {
			new RobobuggyLogicNotification("Something went wrong while waiting...", RobobuggyMessageLevel.WARNING);
		} catch (ParseException e) {
			new RobobuggyLogicNotification("Couldn't parse the file given. Make sure it's a proper JSON file", RobobuggyMessageLevel.EXCEPTION);
		}		
	}
}
