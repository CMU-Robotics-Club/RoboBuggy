package org.roboclub.robobuggy.main;

import java.util.ArrayList;
import java.util.List;
import org.roboclub.robobuggy.drivers.Alice;
import org.roboclub.robobuggy.localization.KalmanFilter;
import org.roboclub.robobuggy.logging.RobotLogger;
import org.roboclub.robobuggy.messages.EncoderMeasurement;
import org.roboclub.robobuggy.messages.GpsMeasurement;
import org.roboclub.robobuggy.messages.ImuMeasurement;
import org.roboclub.robobuggy.messages.SteeringMeasurement;
import org.roboclub.robobuggy.messages.WheelAngleCommand;
import org.roboclub.robobuggy.nodes.EncoderNode;
import org.roboclub.robobuggy.nodes.GpsNode;
import org.roboclub.robobuggy.nodes.ImuNode;
import org.roboclub.robobuggy.nodes.SteeringNode;
import org.roboclub.robobuggy.ros.ActuatorChannel;
import org.roboclub.robobuggy.ros.Message;
import org.roboclub.robobuggy.ros.MessageListener;
import org.roboclub.robobuggy.ros.Node;
import org.roboclub.robobuggy.ros.Publisher;
import org.roboclub.robobuggy.ros.SensorChannel;
import org.roboclub.robobuggy.ros.Subscriber;
import org.roboclub.robobuggy.ui.Gui;

/**
 * @author ??
 * @version 0.0
 * TODO document
 * manages all of the saftey systems of the buggy 
 *
 */
public class Robot implements RosMaster {
	private static Robot instance;
	private static Thread alice;
	private static boolean autonomous;
	private static ArrayList<Node> sensorList;
	private KalmanFilter kf;
	private static Publisher steerPub;
	private static Publisher brakePub;
	
	/**
	 * TODO document
	 * @return
	 */
	public static Robot getInstance() {
		if (instance == null) {
			instance = new Robot();
		}
		return instance;
	}

	/**
	 * TODO document 
	 */
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
			GpsNode gps = new GpsNode(SensorChannel.GPS);
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
			ImuNode imu = new ImuNode(SensorChannel.IMU);
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
			EncoderNode encoder = new EncoderNode(SensorChannel.ENCODER);
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
			SteeringNode controls = new SteeringNode(SensorChannel.DRIVE_CTRL);
			sensorList.add(controls);
			
			new Subscriber(SensorChannel.DRIVE_CTRL.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					updateSteering((SteeringMeasurement)m);
				}
			});
		}

		if (config.VISION_SYSTEM_DEFAULT) {
			System.out.println("Initializing Vision System");
			//vision = new VisionSystem(SensorChannel.VISION);
			System.out.println("JUST KIDDING VISION SYSTEM CANNOT WORK NOW");
			//sensorList.add(vision);
			
			// TODO add subscriber for vision messages
		}

		System.out.println();

		// Start Autonomous Control
		if (autonomous) {
			if (!config.DRIVE_DEFAULT) {
				System.out.println("Initialize Drive Controls for Autonomous");
			} else {
				System.out.println("Alice is in control!");
				alice = new Thread(new Alice());
				alice.start(); 
				
				// Initialize publishers for sending commands
				steerPub = new Publisher(ActuatorChannel.STEERING.getMsgPath());
				brakePub = new Publisher(ActuatorChannel.BRAKE.getMsgPath());
			}
		}
		
		Gui.EnableLogging();
		System.out.println();
	}

	/**
	 * TODO document 
	 * @return
	 */
	public KalmanFilter getKalmanFilter(){
		return kf; 
	}
	
	/**
	 * TODO document 
	 * @return
	 */
	public static ArrayList<Node> getSensorList(){
		return sensorList;
	}
	
	/**
	 * shuts down the robot and all of its child sensors
	 */
	public static void ShutDown() {
		if (sensorList != null && !sensorList.isEmpty()) {
			for (Node sensor : sensorList) {
				if (sensor != null) {
					sensor.shutdown();
				}
			}
		}
		System.exit(0);
	}

	/*public static VisionSystem GetVision() {
		//return vision;
		return null;
	}*/
	
	/**
	 * Methods for Updating Current State 
	 * TODO implement 
	 * @param m
	 **/
	private void updateGps(GpsMeasurement m) {
		// TODO Update planner
	}

	/**
	 * TODO document
	 * TODO implement
	 * @param m
	 */
	private void updateImu(ImuMeasurement m) {
		// TODO Update planner
	}

	/**
	 * TODO document
	 * TODO implement
	 * @param m
	 */
	private void updateSteering(SteeringMeasurement m) {
		// TODO update planner
	}

	/**
	 * TODO document 
	 * TODO implement
	 * @param m
	 */
	private void updateEnc(EncoderMeasurement m) {
		// TODO update planner
	}
	
	/**
	 * TODO document
	 *  Methods for Autonomous Control 
	 */
	public void writeAngle(int angle) {
		if (autonomous) {
			steerPub.publish(new WheelAngleCommand(angle));
		} else {
			System.out.println("Can only control steering in Autonomous mode!");
		}
	}
	
	/**
	 * TODO document 
	 * TODO implment
	 * @param brakesDown
	 */
	public void writeBrakes(boolean brakesDown) {
		if (autonomous) {
			// TODO new pub/sub for command messages
			//brakePub.publish(new BrakeCommand(brakesDown));
		} else {
			System.out.println("Can only control steering in Autonomous mode!");
		}
	}
	
	/**
	 * TODO document 
	 * TODO implment
	 * @return
	 */
	public boolean get_autonomous() {
		return autonomous;
	}
	
	@Override
	/**
	 * TODO document 
	 * TODO implement
	 */
	public List<Node> getAllSensors() {
		return sensorList;
	}

	@Override
	/**
	 * TODO document
	 * TODO implement 
	 */
	public boolean shutDown() {
		// TODO Auto-generated method stub
		return false;
	}
}
