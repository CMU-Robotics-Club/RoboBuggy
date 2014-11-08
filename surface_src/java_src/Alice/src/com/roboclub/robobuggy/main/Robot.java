package com.roboclub.robobuggy.main;

import java.util.ArrayList;
import java.util.Date;

import com.roboclub.robobuggy.localization.KalmanFilter;
import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.sensors.DriveControls;
import com.roboclub.robobuggy.sensors.Encoder;
import com.roboclub.robobuggy.sensors.Gps;
import com.roboclub.robobuggy.sensors.Imu;
import com.roboclub.robobuggy.sensors.Sensor;
import com.roboclub.robobuggy.sensors.SensorState;
import com.roboclub.robobuggy.sensors.VisionSystem;
import com.roboclub.robobuggy.ui.Gui;

public class Robot {
	private static Robot instance;
	private static Thread alice;
	private static boolean autonomous;
	private static ArrayList<Sensor> sensorList;
	private KalmanFilter kf;
	private static VisionSystem vision;
	
	//this moves away from the sensor list model but is needed for the control panel 
	//will not be needed once pub sub is working TODO
	private Gps gps;
	private Imu imu;
	private Encoder encoder;
	private DriveControls controls;
	
	//TODO fix
	public SensorState getVisionState() {
		return SensorState.NOT_IN_USE;
	}
	
	public SensorState getControlInputState(){
		if(controls == null){
			return SensorState.NOT_IN_USE;
		}
		return controls.getState();
	}
	
	public String getControlInputMsg(){
		if(controls == null){
			return "mega not init";
		}
		return Integer.toString(controls.steeringAngle);
	}
	
	public SensorState getGpsState(){
		if(gps == null){
			return SensorState.NOT_IN_USE;
		}
		return gps.getState();
	}
	
	public String getGpsMsg(){
		if(gps == null){
			return "GPS not init";
		}
		return "("+Double.toString(gps.lat) +","+ Double.toString(gps.lon)+")";
	}
	
	public SensorState getEncoderState(){
		if(encoder == null){
			return SensorState.NOT_IN_USE;
		}
		return encoder.getState();
	}
	
	public String getEncoderMsg(){
		if(encoder == null){
			return "encoder not init";
		}
		return "ticks:"+Integer.toString(encoder.getTicks());
	}
	
	public SensorState getImuState(){
		if(imu == null){
			return SensorState.NOT_IN_USE;
		}
		return imu.getState();
	}
	
	public String getImuMsg(){
		if(imu == null){
			return "encoder not init";
		}
		return "th:"+Double.toString(imu.angle);
	}
	
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
			gps = new Gps("/sensors/gps");
			sensorList.add(gps);
		}


		if (config.IMU_DEFAULT) {
			System.out.println("Initializing IMU Serial Connection");
			imu = new Imu("/sensors/imu");
			sensorList.add(imu);
		}

		if (config.ENCODER_DEFAULT) {
			System.out.println("Initializing Encoder Serial Connection");
			encoder = new Encoder("/sensors/encoders");
			sensorList.add(encoder);
		}

		if (config.DRIVE_DEFAULT) {
			System.out.println("Initializing Drive Serial Connection");
			controls = new DriveControls("/sensors/controls");
			sensorList.add(controls);
		}

		if (config.VISION_SYSTEM_DEFAULT) {
			System.out.println("Initializing Vision System");
			vision = new VisionSystem("/sensors/vision");
			sensorList.add(vision);
		}

		System.out.println();

		// Start Autonomous Control
		if (autonomous) {
			System.out.println("Alice is in control!");
			alice = new Thread(new Planner());
			alice.start();
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
	
	/* Methods for Updating Planner, Gui, and Logger */
	public static void UpdateGps(float latitude, float longitude) {
		if (config.logging) {
			// gui.UpdateGps(latitude, longitude);
			RobotLogger rl = RobotLogger.getInstance();
			long time_in_millis = new Date().getTime();
			if(config.active){
				rl.sensor.logGps(time_in_millis, latitude, longitude);
			}
		}

		// TODO Update planner
	}

	public static void UpdateImu(float aX, float aY, float aZ, float rX,
			float rY, float rZ, float mX, float mY, float mZ) {
		if (config.logging) {
			RobotLogger rl = RobotLogger.getInstance();
			float[] acc = {aX, aY, aZ};
			float[] gyro = {rX, rY, rZ};
			float[] compass = {mX, mY, mZ};
			if(config.active){
				rl.sensor.logImu(new Date().getTime(), acc, gyro, compass);
			}
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
			RobotLogger rl = RobotLogger.getInstance();
			if(config.active){
				RobotLogger.sensor.logEncoder(new Date().getTime(),encTick,encReset,encTime);
			}
		}

		// TODO update planner
	}

	public static void UpdateBrake(char angle) {
		if (config.logging) {
			// TODO add logging
		}

		// TODO update planner
	}

	public static void UpdateAngle(int angle) {
		if (config.logging) {
			// TODO add logging
		}

		// TODO Update planner
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
