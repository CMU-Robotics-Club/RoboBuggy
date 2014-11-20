package com.roboclub.robobuggy.main;

import java.io.File;
import java.util.Date;

import com.roboclub.robobuggy.logging.MessageLogWriter;
import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.sensors.FauxArduino;

public class SimRobot {
	// published encoder, subscribes steering and brake
	// TODO: pass the subscribe/pub paths in as arguments?
	private static FauxArduino arduino = new FauxArduino();

	private Subscriber encLogger = new Subscriber("/sensor/encoder",
			new EncLogger());

	public SimRobot() {
		// Stop after 500 feet
	}

	private class EncLogger implements MessageListener {
		MessageLogWriter enc_log = new MessageLogWriter(new File(
				"C:\\Users\\Matt"), new Date());

		public EncLogger() {

		}

		@Override
		public void actionPerformed(String topicName, Message m) {
			System.out.println("received message; loggin!");
			enc_log.log(m);

		}
	}

	public static void UpdateEnc(double distance, double velocity) {
		if (config.logging) {
			RobotLogger rl = RobotLogger.getInstance();
			long time_in_millis = new Date().getTime();
			// rl.sensor.logEncoder(time_in_millis, encTickLast, encReset,
			// encTime);
		}

		// TODO Update planner
	}

	public static void UpdateAngle(int angle) {
		if (config.logging) {
			// TODO add logging
		}
	}
}
