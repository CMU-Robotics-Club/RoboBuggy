package com.roboclub.robobuggy.serial;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.WheelAngleCommand;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * 
 * @author Matt Sebek
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class FauxArduino {

	private int encReset;
	private int encTickLast;
	private int encTime;

	// Set up publishers
	private Publisher encoderPub = new Publisher("/sensor/encoder");
	private Subscriber wheelAngleSub = new Subscriber("/actuator/wheelAngle",
			new wheelAngleCallback());
	private Subscriber brakeSub = new Subscriber("/actuator/brake",
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
			WheelAngleCommand wac = (WheelAngleCommand) m;
			System.out.printf("Wheel commanded to position %d\n", wac.angle);
		}
	}

	public FauxArduino() {
		System.out.println("Initializing Fake Arudino!!");

		(new Thread(new Runnable() {
			@Override
			public void run() {
				int distance = 0;
				int speed = 5;
				while (true) {
					try {
						Thread.sleep(10000);
					} catch (InterruptedException ie) {
						throw new RuntimeException(
								"Sleep should not be throwing");
					}
					distance += speed;
					encoderPub.publish(new EncoderMeasurement(distance, speed));
					System.out.println("sent message");
				}
			}
		})).start();
	}
}
