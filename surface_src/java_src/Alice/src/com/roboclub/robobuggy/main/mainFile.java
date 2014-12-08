package com.roboclub.robobuggy.main;

import java.util.ArrayList;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.FauxGps;
import com.roboclub.robobuggy.sensors.Sensor;
import com.roboclub.robobuggy.ui.Gui;

public class mainFile {
	static Robot buggy;
	
	public static void main(String args[]) {
		//ArrayList<Integer> cameras = new ArrayList<Integer>();  //TODO have this set the cameras to use 
		config.getInstance();//must be run at least once
		
		for (int i = 0; i < args.length; i++) {
			if (args[i].equalsIgnoreCase("-g")) {
				config.GUI_ON = false;
			} else if (args[i].equalsIgnoreCase("+g")) {
				config.GUI_ON = true;
			} else if (args[i].equalsIgnoreCase("-r")) {
				config.active = false;
			} else if (args[i].equalsIgnoreCase("+r")) {
				config.active = true;
			}
		}
		
		if(config.GUI_ON){
			Gui.getInstance();
		}
		
		// Starts the robot
		if(config.DATA_PLAY_BACK_DEFAULT){
			bringup_sim();
		} else {
			Robot.getInstance();
		}	
		
		
	}
	
	public static void bringup_sim() {
		ArrayList<Sensor> sensorList = new ArrayList<Sensor>();

		// Turn on logger!
		if(config.logging){
			System.out.println("Starting Logging");
			RobotLogger.getInstance();
		}

		// Initialize Sensor
		if (config.GPS_DEFAULT) {
			System.out.println("Initializing GPS Serial Connection");
			FauxGps gps = new FauxGps(SensorChannel.GPS);
			sensorList.add(gps);

		}

		Gui.EnableLogging();

		/*if (config.IMU_DEFAULT) {
			System.out.println("Initializing IMU Serial Connection");
			Imu imu = new Imu(SensorChannel.IMU);
			sensorList.add(imu);
			
			new Subscriber(SensorChannel.IMU.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					//updateImu((ImuMeasurement)m);
				}
			});

		}*/
		
	}
}
