package com.roboclub.robobuggy.fauxNodes;

import java.util.Date;

import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;

public class FauxIMUNode extends FauxNode {
	
	private Publisher statePub;
	private Publisher msgPub;
	
	public FauxIMUNode (SensorChannel sensor) {
		typeString = "IMU";
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		statePub.publish(new StateMessage(SensorState.DISCONNECTED));
		statePub.publish(new StateMessage(SensorState.ON));
	}
	
	@Override
	public void parse() {
		new Thread(new Runnable() {
			public void run() {
				while(true) {
					try {
						String toParse = q.take();
						String[] parsed = toParse.split(",");

						//sanity checks
						assert parsed.length == 5;
//						assert parsed[0] == "sensors/imu";
						
						Date date = makeDate(parsed[1]);
						double aX = Double.parseDouble(parsed[2]);
						double aY = Double.parseDouble(parsed[3]);
						double aZ = Double.parseDouble(parsed[4]);
//						double rX = Double.parseDouble(parsed[5]);
//						double rY = Double.parseDouble(parsed[6]);
//						double rZ = Double.parseDouble(parsed[7]);
//						double mX = Double.parseDouble(parsed[8]);
//						double mY = Double.parseDouble(parsed[9]);
//						double mZ = Double.parseDouble(parsed[10]);
						
						//sleep magic here
						sleep(date);
						
						//publish value here
						msgPub.publish(new ImuMeasurement(aX, aY, aZ));
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}	
}