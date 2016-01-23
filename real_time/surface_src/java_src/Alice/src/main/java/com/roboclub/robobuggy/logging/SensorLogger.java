package com.roboclub.robobuggy.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * Logs data from the sensors
 * 
 * @author Joe Doyle
 * @author Vivaan Bahl
 * @author Trevor Decker
 */
public final class SensorLogger {
	private final Queue<String> logQueue;

	private static Queue<String> startLoggingThread(PrintStream stream) {
		final LinkedBlockingQueue<String> ret = new LinkedBlockingQueue<>();
		
		String name = "\"name\": \"Robobuggy Data Logs\",";
		String schemaVersion = "\"schema_version\": 1.0,";
		String dateRecorded = "\"date_recorded\": \"" + 
				new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS").format(new Date()) + "\",";
		String swVersion = "\"software_version\": \"" + getCurrentSoftwareVersion() + "\",";
		String sensorDataHeader = "\"sensor_data\": [";
		stream.println("{" + "\n    " + name + "\n    " + schemaVersion + "\n    " + dateRecorded
				+ "\n    " + swVersion + "\n    " + sensorDataHeader);
		
		Thread loggingThread = new Thread() {
			private int logButtonHits = 0;
			private int gpsHits = 0;
			private int imuHits = 0;
			private int encoderHits = 0;
			private int brakeHits = 0;
			private int steeringHits = 0;
			
			public void run() {
				while (true) {
					try {
						String line = ret.take();
                        System.out.println(line);
                        if (line == null) {
							break;
						}
						if (line.contains("STOP")) {
							stream.println("        " + LoggingNode.translatePeelMessageToJObject(line).toJSONString());
							logButtonHits++;
							stream.println("    ],\n    \"data_breakdown\" : " + getDataBreakdown() + "\n}");
							break;
						}
						stream.println("        " + parseData(line) + ",");
					} catch (InterruptedException e) {
						//TODO add to messages 
						e.printStackTrace();
					}
				}
			}

			@SuppressWarnings("unchecked")
			private String parseData(String line) {
				String sensor = line.substring(line.indexOf("/") + 1, line.indexOf(","));
				JSONObject sensorEntryObject;

				switch (sensor) {
				case "imu":
					sensorEntryObject = ImuNode.translatePeelMessageToJObject(line);
					imuHits++;
					break;
				
				case "gps":
					sensorEntryObject = GpsNode.translatePeelMessageToJObject(line);
					gpsHits++;
					break;
					
				case "steering":
					sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
					break;
				case "fp_hash":
					sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
					break;
				case "commanded_steering":
					sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
					steeringHits++;
					break;
				case "encoder":
					sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
					encoderHits++;
					break;
					
				case "logging_button":
					sensorEntryObject = LoggingNode.translatePeelMessageToJObject(line);
					logButtonHits++;
					break;

				default:
					//put brakes in here?
					sensorEntryObject = new JSONObject();
					sensorEntryObject.put("Unknown Sensor:", sensor);
					break;
				}
				
				return sensorEntryObject.toJSONString();
			}

			@SuppressWarnings("unchecked")
			private String getDataBreakdown() {
				JSONObject dataBreakdownObj = new JSONObject();
				
				dataBreakdownObj.put("logging_button", logButtonHits);
				dataBreakdownObj.put("gps", gpsHits);
				dataBreakdownObj.put("IMU", imuHits);
				dataBreakdownObj.put("encoder", encoderHits);
				dataBreakdownObj.put("brake", brakeHits);
				dataBreakdownObj.put("steering", steeringHits);
				
				return dataBreakdownObj.toJSONString();
			}
		};
		// TODO this potentially prevents a problem where the logger gets CPU
		// starved.
		// the actual fix probably involved throttling the sensors to a
		// reasonable update
		// frequency
		loggingThread.setPriority(10);//TODO this is terible 
		loggingThread.start();
		return ret;
	}

	private static String getCurrentSoftwareVersion() {
		// TODO Auto-generated method stub
		//TODO update this to actually grab current sw version
		return "1.0.0";
	}

	/**
	 * Construct a new {@link SensorLogger} object
	 * @param outputDir {@link File} of the output file directory
	 */
	public SensorLogger(File outputDir) {
		if (outputDir == null) {
			throw new IllegalArgumentException("Output Directory was null!");
		} else if (!outputDir.exists()) {
			if(!outputDir.mkdirs())
				throw new RuntimeException("Failed to create output directory");
		}

		File logFile = new File(outputDir, "sensors.txt");
		System.out.println("FileCreated: " + logFile.getAbsolutePath());
		PrintStream log;
		try {
			log = new PrintStream(logFile, "UTF-8");
		} catch (FileNotFoundException | UnsupportedEncodingException e) {
			e.printStackTrace();
			throw new RuntimeException("Cannot create sensor log file ("
					+ logFile + ")!");
		}
		logQueue = startLoggingThread(log);

		// Subscribe to ALL THE PUBLISHERS
		for (NodeChannel channel : NodeChannel.values()) {
				new Subscriber(channel.getMsgPath(), (topicName, m) -> logQueue.offer(topicName + "," + m.toLogString()));
		}
	}
}
