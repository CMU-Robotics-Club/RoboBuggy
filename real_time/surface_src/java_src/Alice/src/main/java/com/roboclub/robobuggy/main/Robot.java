package com.roboclub.robobuggy.main;

import java.util.LinkedList;
import java.util.List;

import com.roboclub.robobuggy.nodes.sensors.CameraNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;

import java.io.FileNotFoundException;
import java.util.LinkedList;
import java.util.List;

import com.roboclub.robobuggy.nodes.localizers.OdomLocalizer;
import com.roboclub.robobuggy.nodes.planners.DeadreckoningPlanner;
import com.roboclub.robobuggy.nodes.planners.SweepNode;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.simulation.SimulationPlayer;

import java.util.LinkedList;
import java.util.List;
import java.util.LinkedList;
import java.util.List;

//TODO break into a separate util class 

/**
 * 
 * The user interface controls what gui is  displayed to the user and how they can interact with system 
 *
 */
public final class Robot implements RosMaster {

	/************************************** Sets all internal private variables *************************/
	private static final int COMMAND_PERIOD = 50;
	private static Robot instance;
	private boolean autonomous;
	private List<Node> nodeList;
	//TODO find out the actual time we need to put here
	private static final int ARDUINO_BOOTLOADER_TIMEOUT = 2000;

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
		//nodeList.add(new SimulationPlayer());
		nodeList.add(new OdomLocalizer());
		//nodeList.add(new GpsNode(NodeChannel.GPS, RobobuggyConfigFile.COM_PORT_GPS));
		nodeList.add(new ImuNode(NodeChannel.IMU, RobobuggyConfigFile.COM_PORT_IMU));
		nodeList.add(new RBSMNode(NodeChannel.ENCODER, NodeChannel.STEERING, RobobuggyConfigFile.COM_PORT_RBSM, COMMAND_PERIOD));
		nodeList.add(new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, RobobuggyConfigFile.LOG_FILE_LOCATION, NodeChannel.values()));
//		nodeList.add(new SweepNode(NodeChannel.DRIVE_CTRL));
		try {
			nodeList.add(new WayPointFollowerPlanner(NodeChannel.UNKNOWN_CHANNEL,WayPointUtil.CreateWayPointsFromWaypointList("logs/2016-02-13-21-21-41/waypoints.txt")));
		} catch (FileNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

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
