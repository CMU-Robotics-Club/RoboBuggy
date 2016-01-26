package com.roboclub.robobuggy.logging;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurment;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.ui.LocTuple;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;

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
			private int notificationHits = 0;
			
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

			@SuppressWarnings("unchecked")
			private String parseData(String line) {
				String sensor = line.substring(line.indexOf("/") + 1, line.indexOf(","));
				JSONObject sensorEntryObject;
				NodeChannel channelForSensor = NodeChannel.getNodeForName(sensor);

				if (channelForSensor.equals(NodeChannel.UNKNOWN_CHANNEL)) {
					new RobobuggyLogicNotification("Tried to parse an unknown sensor: " + sensor, RobobuggyMessageLevel.WARNING);
					sensorEntryObject = new JSONObject();
					sensorEntryObject.put("Unknown sensor!", sensor);
					return sensorEntryObject.toJSONString();
				}


				switch (channelForSensor) {
					case IMU:
                        sensorEntryObject = ImuNode.translatePeelMessageToJObject(line);
                        imuHits++;
                        break;

					case GPS:
                        sensorEntryObject = GpsNode.translatePeelMessageToJObject(line);
                        gpsHits++;
                        break;

					case STEERING:
                        sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
                        break;

					case FP_HASH:
                        sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
                        break;

					case STEERING_COMMANDED:
                        sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
                        steeringHits++;
                        break;

					case ENCODER:
                        sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
                        encoderHits++;
                        break;

					case GUI_LOGGING_BUTTON:
                        sensorEntryObject = LoggingNode.translatePeelMessageToJObject(line);
                        logButtonHits++;
                        break;

					case LOGIC_EXCEPTION:
						sensorEntryObject = RobobuggyLogicNotificationMeasurment.translatePeelMessageToJObject(line);
						notificationHits++;
						break;

                    default:
                        //put brakes in here?
                        sensorEntryObject = new JSONObject();
                        sensorEntryObject.put("Unknown Sensor:", sensor);
                        break;
				}

				sensorEntryObject.put("name", channelForSensor.getName());
				return sensorEntryObject.toJSONString();
			}

			@SuppressWarnings("unchecked")
			private String getDataBreakdown() {
				JSONObject dataBreakdownObj = new JSONObject();
				
				dataBreakdownObj.put(NodeChannel.GUI_LOGGING_BUTTON.getName(), logButtonHits);
				dataBreakdownObj.put(NodeChannel.GPS.getName(), gpsHits);
				dataBreakdownObj.put(NodeChannel.IMU.getName(), imuHits);
				dataBreakdownObj.put(NodeChannel.ENCODER.getName(), encoderHits);
				dataBreakdownObj.put(NodeChannel.BRAKE.getName(), brakeHits);
				dataBreakdownObj.put(NodeChannel.STEERING.getName(), steeringHits);
				dataBreakdownObj.put(NodeChannel.LOGIC_EXCEPTION, notificationHits);
				
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

	 /**Construct a new {@link SensorLogger} object
	  **@param outputDir {@link File} of the output file directory
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
		Gui.getInstance().getControlPanel().getLoggingPanel().setFileName(outputDir + "/sensors.txt");
		// Subscribe to ALL THE PUBLISHERS
		for(NodeChannel channel : NodeChannel.values()){
				new Subscriber(channel.getMsgPath(),new MessageListener() {
					@Override
					public void actionPerformed(String topicName, Message m) {
						logQueue.offer(topicName + "," + m.toLogString());
					}
				});		
		}

	}
}
