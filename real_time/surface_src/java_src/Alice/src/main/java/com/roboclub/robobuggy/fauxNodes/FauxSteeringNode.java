package com.roboclub.robobuggy.fauxNodes;

import java.util.Date;

import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;

public class FauxSteeringNode extends FauxNode {
	
	private Publisher statePub;
	private Publisher msgPub;
	
	public FauxSteeringNode (SensorChannel sensor) {
		typeString = "DRIVE_CTRL";
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		statePub.publish(new StateMessage(SensorState.DISCONNECTED));
		statePub.publish(new StateMessage(SensorState.ON));
	}
	
	@Override
	public void parse (){
		new Thread(new Runnable() {
			public void run() {
				while(true) {
					try {
						String toParse = q.take();
						String[] parsed = toParse.split(",");

						//sanity checks
						assert parsed.length == 3;
//						assert parsed[0] == "sensors/steering";
						
						Date date = makeDate(parsed[1]);
						double angle = Double.parseDouble(parsed[2]);
						
						//sleep magic here
						sleep(date);
						
//						//publish value here
//						System.out.println(String.format("Steering:      Current Date: %s, Date: %s, Steer Angle: %f",
//								BasicParser.makeStringFromDate(new Date()), BasicParser.makeStringFromDate(date), angle));						
						msgPub.publish(new SteeringMeasurement((int)angle));
						
					} catch (InterruptedException e) {
						System.out.println("Interrupted");
					}
				}
			}
		}).start();
	}
}
