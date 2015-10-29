package com.roboclub.robobuggy.fauxNodes;

import java.util.Date;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;

public class FauxEncoderNode extends FauxNode {
	
	private Publisher statePub;
	private Publisher msgPub;
	
	public FauxEncoderNode (SensorChannel sensor) {
		typeString = "ENC";
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
						assert parsed.length == 6;
//						assert parsed[0] == "sensors/encoder";
						
						Date date = makeDate(parsed[1]);
						double magic1 = Double.parseDouble(parsed[2]);
						double distance = Double.parseDouble(parsed[3]);
						double velocity = Double.parseDouble(parsed[4]);
						double accel = Double.parseDouble(parsed[5]);
												
						//sleep magic here
						sleep(date);
						
						//publish value here
												
						msgPub.publish(new EncoderMeasurement(new Date(), magic1, distance, velocity, accel));
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}
}
