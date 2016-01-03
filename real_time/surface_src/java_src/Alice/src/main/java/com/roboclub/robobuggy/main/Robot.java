package com.roboclub.robobuggy.main;

import java.util.ArrayList;
import java.util.List;

import com.roboclub.robobuggy.localization.KalmanFilter;
import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.RobobuggyLogicExceptionMeasurment;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.messages.WheelAngleCommand;
import com.roboclub.robobuggy.nodes.RBSMNode;
import com.roboclub.robobuggy.nodes.GpsNode;
import com.roboclub.robobuggy.nodes.ImuNode;
import com.roboclub.robobuggy.ros.ActuatorChannel;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.Gui;

/**
 * 
 * The user interface controls what gui is  displayed to the user and how they can interact with system 
 *
 */
public class Robot implements RosMaster {
	

	/************************************** Sets all internal private variables *************************/
	private static final int COMMAND_PERIOD = 1000;
	private static Robot instance;
	private static Thread alice;
	private static boolean autonomous;
	private static ArrayList<Node> sensorList;
	
	/************************************* Set of all public functions ********************************/
	// shuts down the robot and all of its child sensors
	public static void ShutDown() {
		new RobobuggyLogicException("shutting down Robot", MessageLevel.NOTE);
		if (sensorList != null && !sensorList.isEmpty()) {
			for (Node sensor : sensorList) {
				if (sensor != null) {
					sensor.shutdown();
				}
			}
		}
		System.exit(0);
	}

	@Override
	public boolean shutDown() {
		// TODO Auto-generated method stub
		return false;
	}
	
	/************************************* Set of all internal private functions ************************/
	private Robot() {
		System.out.println("Starting Robot");
		autonomous = config.AUTONOMUS_DEFAULT;
		
		RobobuggyLogicException.setupLogicException(SensorChannel.LOGIC_EXCEPTION);
		new RobobuggyLogicException("Logic Exception Setup properly" ,  MessageLevel.NOTE);
		
		// Initialize Nodes
		BuggyNode gps = new GpsNode(SensorChannel.GPS);
		gps.startNode();
		setupIMUNode();
		setupRBSMNode();
		setupVisionNodes();
		setupAutonomusNodes();

	}
	
	public void setupAutonomusNodes(){
		// Start Autonomous Control
		if (autonomous) {
			if (!config.DRIVE_DEFAULT) {
				System.out.println("Initialize Drive Controls for Autonomous");
			} else {
				System.out.println("Alice is in control!");
				alice = new Thread(new Planner());
				alice.start();
				
				// Initialize publishers for sending commands
				steerPub = new Publisher(ActuatorChannel.STEERING.getMsgPath());
				brakePub = new Publisher(ActuatorChannel.BRAKE.getMsgPath());
			}
		}
	}
	
	/***************************************   Getters ********************************/
	public static int getCommandPeriod() {
		return COMMAND_PERIOD;
	}
	
	public boolean get_autonomous() {
		return autonomous;
	}
	
	public static Robot getInstance() {
		if (instance == null) {
			instance = new Robot();
		}
		return instance;
	}
	
	/***************************************  setters ********************************/
}
