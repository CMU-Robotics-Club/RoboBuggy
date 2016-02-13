package com.roboclub.robobuggy.jetty.gui;

import java.io.IOException;
import java.util.Random;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.LinkedBlockingQueue;

import org.eclipse.jetty.websocket.api.Session;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurment;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

public class ClientUpdater {
	
	// Shouldn't actually need to be threadsafe, I don't think it gets accessed anywhere else
	private static LinkedBlockingQueue<String> updates = new LinkedBlockingQueue<String>();
	
	public ClientUpdater() {
		
		// We need to subscribe to all of the nodes first
		// Tested to work with (broken?) sensor playback. For now, we're just going to send hardcoded values.
//		for(NodeChannel channel : NodeChannel.values()){
//			new Subscriber(channel.getMsgPath(),new MessageListener() {
//				@Override
//				public void actionPerformed(String topicName, Message m) {
//					updates.offer(topicName + "," + m.toLogString());
//				}
//			});
//		}
		
		// {"name":"IMU","params":{"roll":1.28,"pitch":-1.56,"yaw":-130.08},"timestamp":"2015-11-21 06:42:24.781"},
		// {"name":"GPS","params":{"raw_gps_lon":7956.54128,"long_direction":"W","latitude":40.440559666666665,"lat_direction":"N","gps_quality":"2","num_satellites":"8","HDOP":1.65,"antenna_altitude":295.6,"longitude":79.94235466666666,"raw_gps_lat":4026.43358},"timestamp":"2015-11-21 06:42:25.138"},
		// {"name":"Steering_steering","params":{"angle":73.0},"timestamp":"2015-11-21 06:42:25.141"},
		// {"name":"Encoder","params":{"acceleration":null,"distance":2031.8228571428572,"dataword":23316.0,"velocity":0.0},"timestamp":"2015-11-21 06:42:25.141"},
		// {"name":"Steering_commanded_steering","params":{"angle":-34.0},"timestamp":"2015-11-21 06:42:25.157"},
		Thread qUpdater = new Thread() {
			
			String[] items = {
					"{\"name\":\"IMU\",\"params\":{\"roll\":%f,\"pitch\":%f,\"yaw\":%f},\"timestamp\":\"2015-11-21 06:42:24.781\"}",
					"{\"name\":\"Encoder\",\"params\":{\"acceleration\":null,\"distance\":2031.8228571428572,\"dataword\":23316.0,\"velocity\":0.0},\"timestamp\":\"2015-11-21 06:42:25.141\"}"
			};
			
			public void run() {
				
				Random random = new Random();
				
				float roll = 42;
				float pitch = 69;
				float yaw = 100;
								
				while (true) {
					
					int index = random.nextInt(items.length);
					switch(index) {
						case 0:
							updates.offer(String.format(items[index], roll, pitch, yaw));
							break;
						case 1:
							updates.offer(items[index]);
							break;
						default:
							System.out.println("fuck");
							break;
					}

					try {
						Thread.sleep(200);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
					
					roll += random.nextFloat() * 4 - 2;
					pitch += random.nextFloat() * 4 - 2;
					yaw += random.nextFloat() * 4 - 2;
				}
			}
		};

		qUpdater.start();
		
		Thread updater = new Thread() {
			
			ConcurrentHashMap<Integer, Session> clients = WSHandler.clients;
			
			public void run() {
				while (true) {
					try {
						
						String update = updates.take();
						Session session;
						
						for (int key : clients.keySet()) {
							session = clients.get(key);
//							session.getRemote().sendString(parseData(update));
							session.getRemote().sendString(update); // for TESTING only
						}
						
//						Thread.sleep(500);
						
					} catch (Exception e) {
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
                        break;

					case GPS:
                        sensorEntryObject = GpsNode.translatePeelMessageToJObject(line);
                        break;

					case STEERING:
                        sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
                        break;

					case FP_HASH:
                        sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
                        break;

					case STEERING_COMMANDED:
                        sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
                        break;

					case ENCODER:
                        sensorEntryObject = RBSMNode.translatePeelMessageToJObject(line);
                        break;

					case GUI_LOGGING_BUTTON:
                        sensorEntryObject = LoggingNode.translatePeelMessageToJObject(line);
                        break;

					case LOGIC_EXCEPTION:
						sensorEntryObject = RobobuggyLogicNotificationMeasurment.translatePeelMessageToJObject(line);
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
		};
		
		updater.start();
	}
}
