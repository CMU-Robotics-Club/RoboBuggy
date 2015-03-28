package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.WheelAngleCommand;
import com.roboclub.robobuggy.ros.ActuatorChannel;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 *
 * @author Matt Sebek
 * @version 0.5
 * @summary Fake sensor for testing purposes
 */

public class FauxArduino {

	private int encReset;
	private int encTickLast;
	private int encTime;

	// Set up publishers
	private Publisher encoderPub = new Publisher(SensorChannel.ENCODER.getMsgPath());
	private Publisher rcSteeringPub = new Publisher(SensorChannel.RC.getMsgPath());

	private Subscriber steeringSub = new Subscriber(ActuatorChannel.STEERING.getMsgPath(),
			new wheelAngleCallback());
	
	private Subscriber brakeSub = new Subscriber(ActuatorChannel.BRAKE.getMsgPath(),
			new brakeCallback());

	private class wheelAngleCallback implements MessageListener {
		@Override
		public void actionPerformed(String topicName, Message m) {
			WheelAngleCommand wac = (WheelAngleCommand) m;
			System.out.printf("Wheel commanded to position %d\n", wac.angle);
		}
	}

	private class brakeCallback implements MessageListener {
		@Override
		public void actionPerformed(String topicName, Message m) {
			BrakeMessage wac = (BrakeMessage) m;
			System.out.printf("Brake commanded to position %d\n", wac.down);
		}
	}

	public FauxArduino() {
		System.out.println("Initializing Fake Arudino!!");

		// Steering thread!
		(new Thread(new Runnable() {
			@Override
			public void run() {
				float delta = (float) 0.05;
				float angle = -90;
				while (true) {
					try {
						// Send a message at 60hz
						Thread.sleep(1000 / 60);
					} catch (InterruptedException ie) {
						throw new RuntimeException(
								"Sleep should not be throwing");
					}
					angle += delta;
					if(Math.abs(angle) > 90.0) {
						delta = -delta;
					}
					rcSteeringPub.publish(new WheelAngleCommand(angle));
				}
			}
		})).start();

		// Encoder Thread!
		(new Thread(new Runnable() {
			@Override
			public void run() {
				int distance = 0;
				int speed = 5;
				while (true) {
					try {
						// Send a message at 60hz
						Thread.sleep(1000 / 60);
					} catch (InterruptedException ie) {
						throw new RuntimeException(
								"Sleep should not be throwing");
					}
					distance += speed;
					encoderPub.publish(new EncoderMeasurement(distance, speed));
				}
			}
		})).start();
		
		System.out.println("and they're off!");
	}
}
