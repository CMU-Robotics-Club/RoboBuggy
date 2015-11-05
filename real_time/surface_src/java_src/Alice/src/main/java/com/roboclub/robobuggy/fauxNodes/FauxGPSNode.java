package com.roboclub.robobuggy.fauxNodes;

import java.util.Date;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;

public class FauxGPSNode extends FauxNode {
	
	private Publisher statePub;
	private Publisher msgPub;
	
	public FauxGPSNode (SensorChannel sensor) {
		typeString = "GPS";
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		statePub.publish(new StateMessage(SensorState.DISCONNECTED));
		statePub.publish(new StateMessage(SensorState.ON));
	}
	
	@Override
	public void parse () {
		new Thread(new Runnable() {
			public void run() {
				while(true) {
					try {
						String toParse = q.take();
						String[] parsed = toParse.split(",");

						//sanity checks
						assert parsed.length == 11;
//						assert parsed[0] == "sensors/gps";
						
						Date date = makeDate(parsed[1]);
						Date otherDate = makeDate(parsed[2]);
						double lat = Double.parseDouble(parsed[3]);
						String latDir = parsed[4];
						double longi = Double.parseDouble(parsed[5]);
						String longiDir = parsed[6];
						double magic1 = Double.parseDouble(parsed[7]);
						double magic2 = Double.parseDouble(parsed[8]);
						double magic3 = Double.parseDouble(parsed[9]);
						double magic4 = Double.parseDouble(parsed[10]);
												
						//sleep magic here
						sleep(date);
						
						//publish value here
//						System.out.println(String.format("GPS:           Current Date: %s, Date: %s, Other Date: %s, Lat: %s,"
//								+ " %s Long: %s %s %f %f %f %f", BasicParser.makeStringFromDate(new Date()), BasicParser.makeStringFromDate(date),
//								BasicParser.makeStringFromDate(otherDate), lat, latDir, longi, longiDir, magic1, magic2, magic3, magic4));
						msgPub.publish(new GpsMeasurement(otherDate, lat, latDir == "N", longi, longiDir == "W",
								(int)magic1, (int)magic2, magic3, magic4));
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}
}
