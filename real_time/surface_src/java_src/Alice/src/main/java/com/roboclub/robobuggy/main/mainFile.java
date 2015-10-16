package com.roboclub.robobuggy.main;

import java.nio.file.Paths;

import com.roboclub.robobuggy.sensors.SensorManager;
import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.Gui;

public class mainFile {
	static Robot buggy;
	static int num = 0;
	
	public static void main(String args[]) {
		//ArrayList<Integer> cameras = new ArrayList<Integer>();  //TODO have this set the cameras to use 
		config.getInstance();//must be run at least once
		
		System.out.println(Paths.get("").toAbsolutePath().toString());
		System.err.println(Paths.get("").toAbsolutePath().toString());
		
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
			try {
				bringup_sim();
			} catch (Exception e) {
				Gui.close();
				System.out.println("Unable to bringup simulated robot. Stacktrace omitted because it's really big.");
				e.printStackTrace();
				return;
			}
		} else {
			Robot.getInstance();
		}	
	}
	
	public static void bringup_sim() throws Exception {
		// Turn on logger!
		if(config.logging){
			System.out.println("Starting Logging");
			RobotLogger.getInstance();
		}

		Gui.EnableLogging();
		SensorManager sm = SensorManager.getInstance();
		
		String path = "logs/2015-10-04-07-15-51/sensors.txt";
		sm.newFauxSensors(path,
				SensorChannel.IMU
				,SensorChannel.GPS
				,SensorChannel.DRIVE_CTRL
				,SensorChannel.ENCODER
				);
		
		new Subscriber(SensorChannel.ENCODER.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				//System.out.println(m.toLogString());
			}
		});
	}
}
