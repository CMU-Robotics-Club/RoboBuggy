package com.roboclub.robobuggy.main;

import java.util.ArrayList;
import com.roboclub.robobuggy.localization.KalmanFilter;
import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.CommandChannel;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.sensors.DriveControls;
import com.roboclub.robobuggy.sensors.Encoder;
import com.roboclub.robobuggy.sensors.Gps;
import com.roboclub.robobuggy.sensors.Imu;
import com.roboclub.robobuggy.sensors.Sensor;
import com.roboclub.robobuggy.sensors.VisionSystem;
import com.roboclub.robobuggy.ui.Gui;

public class Robot {
	private static Robot instance;
	private static Thread alice;
	private static boolean autonomous;
	private static ArrayList<Sensor> sensorList;
	private KalmanFilter kf;
	private static VisionSystem vision;
	private static Publisher steerPub;
	private static Publisher brakePub;
	
	public static Robot getInstance() {
		if (instance == null) {
			instance = new Robot();
		}
		return instance;
	}

	private Robot() {
		sensorList = new ArrayList<>();
		kf = new KalmanFilter();
		System.out.println("Starting Robot");
		autonomous = config.AUTONOMUS_DEFAULT;

		//creates a log file even if no data is used
		if(config.logging){
			System.out.println("Starting Logging");
			RobotLogger.getInstance();
		}
		
		System.out.println();
		
		// Initialize Sensor
		if (config.GPS_DEFAULT) {
			System.out.println("Initializing GPS Serial Connection");
			Gps gps = new Gps(SensorChannel.GPS);
			sensorList.add(gps);
			
			new Subscriber(SensorChannel.GPS.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					updateGps((GpsMeasurement)m);
				}
			});
		}


		if (config.IMU_DEFAULT) {
			System.out.println("Initializing IMU Serial Connection");
			Imu imu = new Imu(SensorChannel.IMU);
			sensorList.add(imu);
			
			new Subscriber(SensorChannel.IMU.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					updateImu((ImuMeasurement)m);
				}
			});
		}

		if (config.ENCODER_DEFAULT) {
			System.out.println("Initializing Encoder Serial Connection");
			Encoder encoder = new Encoder(SensorChannel.ENCODER);
			sensorList.add(encoder);
			
			new Subscriber(SensorChannel.ENCODER.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					updateEnc((EncoderMeasurement)m);
				}
			});
		}

		if (config.DRIVE_DEFAULT) {
			System.out.println("Initializing Drive Serial Connection");
			DriveControls controls = new DriveControls(SensorChannel.DRIVE_CTRL);
			sensorList.add(controls);
			
			// TODO add subscriber or publisher based on auto or not?
		}

		if (config.VISION_SYSTEM_DEFAULT) {
			System.out.println("Initializing Vision System");
			vision = new VisionSystem(SensorChannel.VISION);
			sensorList.add(vision);
			
			// TODO add subscriber for vision messages
		}

		System.out.println();

		// Start Autonomous Control
		if (autonomous) {
			if (!config.DRIVE_DEFAULT) {
				System.out.println("Initialize Drive Controls for Autonomous");
			} else {
				System.out.println("Alice is in control!");
				alice = new Thread(new Planner());
				alice.start();
				
				// Initialize publishers for sending commands
				steerPub = new Publisher(CommandChannel.STEERING.getMsgPath());
				brakePub = new Publisher(CommandChannel.BRAKE.getMsgPath());
			}
		}
		
		Gui.EnableLogging();
		System.out.println();
	}

	public KalmanFilter getKalmanFilter(){
		return kf; 
	}
	
	public static ArrayList<Sensor> getSensorList(){
		return sensorList;
	}
	
	// shuts down the robot and all of its child sensors
	public static void ShutDown() {
		for (Sensor thisSensor : sensorList) {
			thisSensor.close();
		}
		System.exit(0);
	}

	public static VisionSystem GetVision() {
		return vision;
	}
	
	/* Methods for Updating Current State */
	private void updateGps(GpsMeasurement m) {
		// TODO Update planner
	}

	private void updateImu(ImuMeasurement m) {
		// TODO Update planner
	}

	private void updateSteering(SteeringMeasurement m) {
		// TODO update planner
	}

	private void updateEnc(EncoderMeasurement m) {
		// TODO update planner
	}
	
	public boolean get_autonomous() {
		return autonomous;
	}
	
	public static void main(String args[]) {
		config.getInstance();//must be run at least once
		for (int i = 0; i < args.length; i++) {
			if (args[i].equalsIgnoreCase("-g")) {
				config.GUI_ON = false;
			} else if (args[i].equalsIgnoreCase("+g")){
				config.GUI_ON = true;
			} else if (args[i].equalsIgnoreCase("-r")) {
				config.active = false;
			} else if (args[i].equalsIgnoreCase("+r")){
				config.active = true;
			} else if (args[i].equalsIgnoreCase("-config")) {
				if (i+1 < args.length) {
					config.Set(args[++i]);
				}
			}
		}
		
		if(config.GUI_ON){
			Gui.getInstance();
		}
		//starts the robot
		if(config.DATA_PLAY_BACK_DEFAULT){
			new SimRobot();
		}else{
			Robot.getInstance();
		}	
	}
}
