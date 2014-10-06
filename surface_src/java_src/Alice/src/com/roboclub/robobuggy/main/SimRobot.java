package com.roboclub.robobuggy.main;

import java.util.Date;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.sensors.FauxArduino;

public class SimRobot {
	// published encoder, subscribes steering and brake
	// TODO: pass the subscribe/pub paths in as arguments?
	private static FauxArduino arduino = new FauxArduino();
	
	public SimRobot() {
		// Stop after 500 feet
		
	}
	
	
	public static void UpdateEnc(double distance, double velocity) {
		if (config.logging) {
			RobotLogger rl = RobotLogger.getInstance();
		    long time_in_millis = new Date().getTime();
		    //rl.sensor.logEncoder(time_in_millis, encTickLast, encReset, encTime);
		}
		
		//TODO Update planner
	}
	
	public static void UpdateAngle(int angle) {
		if (config.logging) {
			//TODO add logging
		}
		
	}
}
