package com.roboclub.robobuggy.logging;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
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
	private final PrintStream log;
	private final Queue<String> logQueue;
	private final ArrayList<Subscriber> subscribers;

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

			private String parseData(String line) {
				// TODO Auto-generated method stub
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
				case "fp_hash":
				case "commanded_steering":
					steeringHits++;
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
				// TODO Auto-generated method stub
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
	};

	private static String getCurrentSoftwareVersion() {
		// TODO Auto-generated method stub
		//TODO update this to actually grab current sw version
		return "1.0.0";
	}

	private SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");

	/**
	 * Construct a new {@link SensorLogger} object
	 * @param outputDir {@link File} of the output file directory
	 * @param startTime {@link Date} of the start time of the logger
	 */
	public SensorLogger(File outputDir, Date startTime) {
		if (outputDir == null) {
			throw new IllegalArgumentException("Output Directory was null!");
		} else if (!outputDir.exists()) {
			outputDir.mkdirs();
		}

		File logFile = new File(outputDir, "sensors.txt");
		System.out.println("FileCreated: " + logFile.getAbsolutePath());
		try {
			log = new PrintStream(logFile);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException("Cannot create sensor log file ("
					+ logFile + ")!");
		}
		logQueue = startLoggingThread(log);

		// Subscribe to ALL THE PUBLISHERS
		subscribers = new ArrayList<Subscriber>();
		for (NodeChannel channel : NodeChannel.values()) {
			subscribers.add(
				new Subscriber(channel.getMsgPath(), new MessageListener() {
					@Override
					public void actionPerformed(String topicName, Message m) {
						logQueue.offer(topicName + "," + m.toLogString());
					}
				}));
		}
	}
}
