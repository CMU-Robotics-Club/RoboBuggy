package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.RemoteWheelAngleRequest;
import com.roboclub.robobuggy.messages.WheelAngleCommand;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

public class SimAllSensors {

	// Set up publishers
	private Publisher encoderPub = new Publisher("/sensor/encoder");
	private Publisher gpsPub = new Publisher("/sensor/gps");
	private Publisher imuPub = new Publisher("/sensor/imu");
	private Publisher reqAnglePub = new Publisher("/sensor/requested_angle");

	private Subscriber wheelAngleSub = new Subscriber("/actuator/wheelAngle",
			new wheelAngleCallback());
	private Subscriber brakeSub = new Subscriber("/actuator/brake",
			new brakeCallback());

	float wheel_angle = 0;
	float brake_down = 0;

	// Silly simulator
	float x = 0;
	float y = 0;
	float theta = 0; // wrapped to 0, 2pi
	long time = 0;

	public SimAllSensors() {
		System.out.println("Simulated Buggy Initialized!!");

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

	private void step() {
		// Update internauul state
		// Publish to every sensor
		while (true) {
			encoderPub.publish(new EncoderMeasurement(10, 2));
			//TODO set the correct values 
			gpsPub.publish(new GpsMeasurement(null, null, 42.00f, false, -76.00f, false, 0, 0, brake_down, brake_down));
			//TODO fix imuPub for sim
			//imuPub.publish(new ImuMeasurement(0, 0, 1, 2, 3, 4, 5, 6, 7));
			reqAnglePub.publish(new RemoteWheelAngleRequest(0.5));
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	// Wheel Angle Actuator Callback
	private class wheelAngleCallback implements MessageListener {
		@Override
		public void actionPerformed(String topicName, Message m) {
			WheelAngleCommand wac = (WheelAngleCommand) m;
			System.out.printf("Wheel commanded to position %d\n", wac.angle);
		}
	}

	// Brake Actuator Callback
	private class brakeCallback implements MessageListener {
		@Override
		public void actionPerformed(String topicName, Message m) {
			BrakeMessage bc = (BrakeMessage) m;
			System.out.printf("Wheel commanded to position %d\n", bc.down);
		}
	}

}
