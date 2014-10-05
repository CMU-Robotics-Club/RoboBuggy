package com.roboclub.robobuggy.main;

import java.util.ArrayList;
import java.util.Date;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.sensors.Arduino;
import com.roboclub.robobuggy.sensors.Gps;
import com.roboclub.robobuggy.sensors.Imu;
import com.roboclub.robobuggy.sensors.VisionSystem;
import com.roboclub.robobuggy.ui.Gui;

public class Robot {
	private static Robot instance;
	private static Gui gui;
	private static Thread alice;
	private static Arduino arduino;
	private static Gps gps;
	private static Imu imu;
	private static boolean logging;
	private static boolean running;
	private static VisionSystem vision;
	//private ArrayList<Markers> markers
	

	public static Robot getInstance(){
		if(instance == null){
			instance = new Robot();
		}
		return instance;
	}
	
	
	public Robot() {
		logging = config.LOGGING_DEFAULT;
		running = config.RUNNING_DEFAULT;
		
		
		//TODO break apart the arduino
		Robot.arduino = new Arduino();

		// Initialize Sensor
		if(config.GPS_DEFAULT) gps = new Gps();
		if(config.IMU_DEFAULT) imu = new Imu();
		if(config.VISION_SYSTEM_DEFAULT) vision = new VisionSystem();
		
		//if ((encAng.isConnected() || gps.isConnected() || imu.isConnected()) && logging) {
		if (logging) {
			Robot.gui = new Gui(Robot.arduino, Robot.gps, Robot.imu);
		}
		
		// Start Autonomous Control
		if (running) {
			System.out.println("Alice is in control!");
			alice = new Thread(new Planner());
			alice.start();
		}
	}
	
	//shuts down the robot and all of its child sensors 
	public static void ShutDown() {
		arduino.close();
		gps.close();
		imu.close();
		
		System.exit(0);
	}
	
	/* Methods for Writing to Arduino */
	public static void WriteAngle(int angle) {
		arduino.writeAngle(angle);
	}
	
	public static void WriteBrakes(int angle) {
		arduino.writeBrake(angle);
	}
	
	/* Methods for Updating Planner, Gui, and Logger */
	public static void UpdateGps(float latitude, float longitude) {
		if (logging) {
			//gui.UpdateGps(latitude, longitude);
			RobotLogger rl = RobotLogger.getInstance();
		    long time_in_millis = new Date().getTime();
		    rl.sensor.logGps(time_in_millis, latitude, longitude);
		}
		
		//TODO Update planner
	}
	
	public static void UpdateImu(float aX, float aY, float aZ, 
			float rX, float rY, float rZ, float mX, float mY, float mZ) {
		if (logging) {
			RobotLogger rl = RobotLogger.getInstance();
		    long time_in_millis = new Date().getTime();
		    float[] acc = new float[3];
		    float[] gyro = new float[3];
		    float[] compass = new float[3];
		    acc[0] = aX; acc[1] = aY; acc[2] = aZ;
		    gyro[0] = rX; gyro[1] = rY; gyro[2] = rZ;
		    compass[0] = mX; compass[1] = mY; compass[2] = mZ;
		    rl.sensor.logImu(time_in_millis, acc, gyro, compass);
		}
		
		//TODO Update planner
	}
	
	public static void UpdateSteering(char angle) {
		if (logging) {
			//TODO add logging
		}
		
		//TODO update planner
	}
	
	public static void UpdateError(int error) {
		if (logging) {
			//TODO add logging
		}
		
		//TODO update planner
	}
	
	public static void UpdateEnc(int encTime, int encReset, int encTick) {
		if (logging) {
			//TODO add logging
		}
		
		//TODO update planner
	}
	
	public static void UpdateBrake(char angle) {
		if (logging) {
			//TODO add logging
		}
		
		//TODO update planner
	}
	
	public static void UpdateEnc(double distance, double velocity) {
		if (logging) {
			RobotLogger rl = RobotLogger.getInstance();
		    long time_in_millis = new Date().getTime();
		    //rl.sensor.logEncoder(time_in_millis, encTickLast, encReset, encTime);
		}
		
		//TODO Update planner
	}
	
	public static void UpdateAngle(int angle) {
		if (logging) {
			//TODO add logging
		}
		
		//TODO Update planner
	}
}
