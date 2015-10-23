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
import com.roboclub.robobuggy.nodes.GpsNode;
import com.roboclub.robobuggy.nodes.ImuNode;
import com.roboclub.robobuggy.nodes.LoggingNode;
import com.roboclub.robobuggy.nodes.RBSMNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.serial.SerialNode;

/**
 * Logs data from the sensors
 * 
 * @author Joe Doyle
 * @author Trevor Decker
 */
public final class SensorLogger {
	private final PrintStream _log;
	private final Queue<String> _logQueue;
	private final ArrayList<Subscriber> subscribers;

	private static final Queue<String> startLoggingThread(PrintStream stream) {
		final LinkedBlockingQueue<String> ret = new LinkedBlockingQueue<>();
		
		String name = "\"name\": \"Robobuggy Data Logs\",";
		String schema_version = "\"schema_version\": 1.0,";
		String date_recorded = "\"date_recorded\": \"" + new SimpleDateFormat("MM/dd/yyyy").format(new Date()) + "\",";
		String swVersion = "\"software_version\": " + getCurrentSoftwareVersion() + "\",";
		String sensorDataHeader = "\"sensor_data\":\n[";
		stream.println("{\n" + name + schema_version + date_recorded + swVersion + "\n" + sensorDataHeader);
		
		Thread logging_thread = new Thread() {
			int logButtonHits = 0;
			int gpsHits = 0;
			int imuHits = 0;
			int encoderHits = 0;
			int brakeHits = 0;
			int steeringHits = 0;
			
			public void run() {
				while (true) {
					try {
						String line = ret.take();
						if (line == null) {
							break;
						}
						if (line.contains(config.LOG_STOP_MESSAGE)) {
							stream.println("{ \"timestamp\":\"" + new SimpleDateFormat("HH:mm:ss.SSS").format(new Date()));
							stream.println("    ],\n" + getDataBreakdown() + "}");
							break;
						}
						stream.println(parseData(line));
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
					sensorEntryObject = SerialNode.translatePeelMessageToJObject(line);
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
		logging_thread.setPriority(10);//TODO this is terible 
		logging_thread.start();
		return ret;
	};

	private static String getCurrentSoftwareVersion() {
		// TODO Auto-generated method stub
		//TODO update this to actually grab current sw version
		return "1.0.0";
	}

	SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");

	public SensorLogger(File outputDir, Date startTime) throws Exception {
		if (outputDir == null) {
			throw new Exception("Output Directory was null!");
		} else if (!outputDir.exists()) {
			outputDir.mkdirs();
		}

		File logFile = new File(outputDir, "sensors.txt");
		System.out.println("FileCreated: " + logFile.getAbsolutePath());
		try {
			_log = new PrintStream(logFile);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			throw new RuntimeException("Cannot create sensor log file ("
					+ logFile + ")!");
		}
		_logQueue = startLoggingThread(_log);

		// Subscribe to ALL THE PUBLISHERS
		subscribers = new ArrayList<Subscriber>();
		for (SensorChannel channel : SensorChannel.values()) {
			subscribers.add(
				new Subscriber(channel.getMsgPath(), new MessageListener() {
					@Override
					public void actionPerformed(String topicName, Message m) {
						_logQueue.offer(topicName + "," + m.toLogString());
					}
				}));
		}
	}
}
