package com.roboclub.robobuggy.main;

import java.util.LinkedList;
import java.util.List;

import com.roboclub.robobuggy.nodes.sensors.CameraNode;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.util.LinkedList;
import java.util.List;

/**
 * 
 * The user interface controls what gui is  displayed to the user and how they can interact with system 
 *
 */
public final class Robot implements RosMaster {

	/************************************** Sets all internal private variables *************************/
	private static final int COMMAND_PERIOD = 1000;
	private static final int ARDUINO_BOOTLOADER_TIMEOUT = 2000;
	private static Robot instance;
	private boolean autonomous;
	private List<Node> nodeList;
	
	/************************************* Set of all public functions ********************************/

	/**{@inheritDoc}*/
	@Override
	public boolean shutDown() {
		new RobobuggyLogicNotification("Shutting down Robot", RobobuggyMessageLevel.WARNING);
		return nodeList.stream().map(n -> n.shutdown()).reduce(true, (a,b) -> a&&b);
	}
	
	/**
	 * Starts all of the nodes of the {@link Robot}
	 * @return true iff all the nodes are started successfully
	 */
	@Override
	public boolean startNodes() {
		return nodeList.stream().map(n -> n.startNode()).reduce(true, (a,b) -> a&&b);
	}
	
	/************************************* Set of all internal private functions ************************/
	private Robot() {
		System.out.println("Starting Robot");
		autonomous = RobobuggyConfigFile.AUTONOMOUS_DEFAULT;
		nodeList = new LinkedList<>();
		new RobobuggyLogicNotification("Logic Exception Setup properly" ,  RobobuggyMessageLevel.NOTE);
		// Initialize Nodes
		//nodeList.add(new GpsNode(NodeChannel.GPS, RobobuggyConfigFile.COM_PORT_GPS));
		//nodeList.add(new ImuNode(NodeChannel.IMU, RobobuggyConfigFile.COM_PORT_IMU));
		//nodeList.add(new RBSMNode(NodeChannel.ENCODER, NodeChannel.STEERING,
		//		RobobuggyConfigFile.COM_PORT_RBSM, COMMAND_PERIOD));
		nodeList.add(new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, RobobuggyConfigFile.LOG_FILE_LOCATION, NodeChannel.values()));
		nodeList.add(new CameraNode(NodeChannel.PUSHBAR_CAMERA, 100));
//		nodeList.add(new GPSTrackPlannerNode(NodeChannel.BRAKE_CTRL,
//				RobobuggyConfigFile.LOG_FILE_LOCATION));
//		nodeList.add(new GPSLocalizer(NodeChannel.POSE));
		try {
			Thread.sleep(ARDUINO_BOOTLOADER_TIMEOUT);
		} catch (InterruptedException e) {
			new RobobuggyLogicNotification("Couldn't wait for bootloader, shutting down", RobobuggyMessageLevel.EXCEPTION);
			shutDown();
		}
	}
	
	/***************************************   Getters ********************************/
	/**
	 * Returns the period at which commands are sent to the actuators
	 * @return the period at which commands are sent to the actuators
	 */
	public static int getCommandPeriod() {
		return COMMAND_PERIOD;
	}
	
	/**
	 * Returns true iff the robot has autonomous mode enabled
	 * @return true iff the robot has autonomous mode enabled
	 */
	public boolean getAutonomous() {
		return autonomous;
	}
	
	/**
	 * Returns the list of current nodes 
	 * @return the nodeList
	 */ 
	public List<Node> getNodeList(){
		return nodeList;
	}
	
	/**
	 * Returns a reference to the one instance of the {@link Robot} object.
	 * If no instance exists, a new one is created.
	 * @return a reference to the one instance of the {@link Robot} object
	 */
	public static Robot getInstance() {
		if (instance == null) {
			instance = new Robot();
		}
		return instance;
	}

	/**{@inheritDoc}*/
	@Override
	public List<Node> getNodes() {
		return nodeList;
	}
	
	/***************************************  setters ********************************/
}
