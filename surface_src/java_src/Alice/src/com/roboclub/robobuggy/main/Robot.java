package com.roboclub.robobuggy.main;

import java.util.ArrayList;
import java.util.Date;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.sensors.*;
import com.roboclub.robobuggy.serial.Arduino;

public class Robot {
	private static Robot instance;
	private static Thread alice;
	private static Arduino arduino;
	private static boolean autonomous;
	private static ArrayList<Sensor> sensorList;

	// private ArrayList<Markers> markers

	public static Robot getInstance() {
		if (instance == null) {
			instance = new Robot();
		}
		return instance;
	}

	private Robot() {
		sensorList = new ArrayList<>();
		System.out.println("starting Robot");
		autonomous = config.AUTONOMUS_DEFAULT;

		// TODO break apart the arduino
		Robot.arduino = Arduino.getInstance();

		// Initialize Sensor
		if (config.GPS_DEFAULT)
		{
			GPS gps = new GPS("/sensors/GPS");
		    sensorList.add(gps);
		}
		
		if (config.IMU_DEFAULT)
		{
			IMU imu = new IMU("/sensors/IMU");
			sensorList.add(imu);
		}
		
		if(config.ENCODER_DEFAULT)
		{
			Encoder encoder = new Encoder("/sensors/Encoder");
			sensorList.add(encoder);
		}
		
		if (config.VISION_SYSTEM_DEFAULT){
			VisionSystem vision = new VisionSystem("/sensors/vision");
		    sensorList.add(vision);
		}
		
		    // if ((encAng.isConnected() || gps.isConnected() || imu.isConnected())
		// && logging) {
		if (config.active) {
			// Robot.gui = new Gui(Robot.arduino, Robot.gps, Robot.imu);
		}

		// Start Autonomous Control
		if (autonomous) {
			System.out.println("Alice is in control!");
			alice = new Thread(new Planner());
			alice.start();
		}
	}

	// shuts down the robot and all of its child sensors
	public static void ShutDown() {
		for(Sensor thisSensor:sensorList)
		{
			thisSensor.close();
		}
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
		if (config.logging) {
			// gui.UpdateGps(latitude, longitude);
			RobotLogger rl = RobotLogger.getInstance();
			long time_in_millis = new Date().getTime();
			rl.sensor.logGps(time_in_millis, latitude, longitude);
		}

		// TODO Update planner
	}

	public static void UpdateImu(float aX, float aY, float aZ, float rX,
			float rY, float rZ, float mX, float mY, float mZ) {
		if (config.logging) {
			RobotLogger rl = RobotLogger.getInstance();
			long time_in_millis = new Date().getTime();
			float[] acc = new float[3];
			float[] gyro = new float[3];
			float[] compass = new float[3];
			acc[0] = aX;
			acc[1] = aY;
			acc[2] = aZ;
			gyro[0] = rX;
			gyro[1] = rY;
			gyro[2] = rZ;
			compass[0] = mX;
			compass[1] = mY;
			compass[2] = mZ;
			rl.sensor.logImu(time_in_millis, acc, gyro, compass);
		}

		// TODO Update planner
	}

	public static void UpdateSteering(char angle) {
		if (config.logging) {
			// TODO add logging
		}

		// TODO update planner
	}

	public static void UpdateError(int error) {
		if (config.logging) {
			// TODO add logging
		}

		// TODO update planner
	}

	public static void UpdateEnc(int encTime, int encReset, int encTick) {
		if (config.logging) {
			// TODO add logging
		}

		// TODO update planner
	}

	public static void UpdateBrake(char angle) {
		if (config.logging) {
			// TODO add logging
		}

		// TODO update planner
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

		// TODO Update planner
	}

	public boolean get_autonomus() {
		return autonomous;
	}

}
