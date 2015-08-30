package org.roboclub.robobuggy.main;

import java.io.File;
import java.util.Date;

import org.roboclub.robobuggy.logging.MessageLogWriter;
import org.roboclub.robobuggy.logging.RobotLogger;
import org.roboclub.robobuggy.ros.Message;
import org.roboclub.robobuggy.ros.MessageListener;
import org.roboclub.robobuggy.ros.Subscriber;
import org.roboclub.robobuggy.sensors.FauxArduino;

/**
 * @author Matt Sebeck
 * @version 0.0
 * 
 * TODO document 
 *
 */
public class SimRobot {
	/**
	 *  published encoder, subscribes steering and brake
	 *   TODO: pass the subscribe/pub paths in as arguments?
	 */
	private static FauxArduino arduino = new FauxArduino();
	private Subscriber encLogger = new Subscriber("/sensor/encoder",
			new EncLogger());

	/**
	 * TDOO document 
	 * TOD implment
	 */
	public SimRobot() {
		// Stop after 500 feet
	}

	/**
	 * TODO documnet 
	 * TODO break out
	 *
	 */
	private class EncLogger implements MessageListener {
		MessageLogWriter enc_log = new MessageLogWriter(new File(
				"C:\\Users\\Matt"), new Date());

		/**
		 * TODO document 
		 * TODO implment 
		 */
		public EncLogger() {

		}

		@Override
		/**
		 * TODO docuemt 
		 * TODO implement 
		 */
		public void actionPerformed(String topicName, Message m) {
			// System.out.println("received message; loggin!");
			// enc_log.log(m);

		}
	}

	/**
	 * TODO document 
	 * @param distance
	 * @param velocity
	 */
	public static void UpdateEnc(double distance, double velocity) {
		if (config.logging) {
			RobotLogger rl = RobotLogger.getInstance();
			long time_in_millis = new Date().getTime();
			// rl.sensor.logEncoder(time_in_millis, encTickLast, encReset,
			// encTime);
		}

		// TODO Update planner
	}

	/**
	 * TODO documet 
	 * @param angle
	 */
	public static void UpdateAngle(int angle) {
		if (config.logging) {
			// TODO add logging
		}
	}
}
