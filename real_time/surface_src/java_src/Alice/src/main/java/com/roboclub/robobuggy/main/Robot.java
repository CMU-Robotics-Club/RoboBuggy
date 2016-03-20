package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.sensors.CameraNode;
import com.roboclub.robobuggy.nodes.planners.SweepNode;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;

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
	private static Robot instance;
	private boolean autonomous;
	private boolean collectingData;
	private static List<Node> nodeList;
	//TODO find out the actual time we need to put here
	private static final int ARDUINO_BOOTLOADER_TIMEOUT = 2000;

	/************************************* Set of all public functions ********************************/

	/**{@inheritDoc}*/
	//only one instance of a robot will ever exist, however shutdown cannot be static because the method is a part of the ROSMaster interface", 
	@Override
	public synchronized boolean shutDown() {
		new RobobuggyLogicNotification("Shutting down Robot", RobobuggyMessageLevel.WARNING);
		boolean result = nodeList.stream().map(n -> n.shutdown()).reduce(true, (a,b) -> a&&b);
		invalidateInstance();
		return result;
	}
	
	private static synchronized void invalidateInstance(){
		instance = null;
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
	private Robot(boolean collectingData) {
		System.out.println("Starting Robot");
		this.collectingData = collectingData;  //tells us to add or not add online sensors 
		try {
			Thread.sleep(ARDUINO_BOOTLOADER_TIMEOUT);
		} catch (InterruptedException e) {
			new RobobuggyLogicNotification("Couldn't wait for bootloader, shutting down", RobobuggyMessageLevel.EXCEPTION);
			shutDown();
		}
		autonomous = RobobuggyConfigFile.AUTONOMOUS_DEFAULT;
		nodeList = new LinkedList<>();
		new RobobuggyLogicNotification("Logic Exception Setup properly" ,  RobobuggyMessageLevel.NOTE);
		// Initialize Nodes
		
	//	nodeList.add(new SimulationPlayer());
		//nodeList.add(new OdomLocalizer());
		nodeList.add(new HighTrustGPSLocalizer());
		
	//	nodeList.add(new OdomLocalizer());
		
		//this subset of nodes is only added when we are in online mode 
		if(collectingData){
//			nodeList.add(new GpsNode(NodeChannel.GPS, RobobuggyConfigFile.getComPortGPS()));
//			nodeList.add(new ImuNode(NodeChannel.IMU, RobobuggyConfigFile.getComPortImu()));
			nodeList.add(new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, RobobuggyConfigFile.LOG_FILE_LOCATION,
					NodeChannel.getLoggingChannels()));
			nodeList.add(new RBSMNode(NodeChannel.ENCODER, NodeChannel.STEERING, RobobuggyConfigFile.getComPortRBSM(),
					RobobuggyConfigFile.RBSM_COMMAND_PERIOD));
		}
		

		nodeList.add(new SweepNode(NodeChannel.DRIVE_CTRL));

	/*
		try {
		nodeList.add(new WayPointFollowerPlanner(NodeChannel.UNKNOWN_CHANNEL,
					WayPointUtil.createWayPointsFromLog("logs/", "spring1data/2016-02-20-06-50-45/sensors_2016-02-20-06-50-45.txt")));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

*/

//		nodeList.add(new CameraNode(NodeChannel.PUSHBAR_CAMERA, 100));
//		nodeList.add(new GPSTrackPlannerNode(NodeChannel.BRAKE_CTRL,RobobuggyConfigFile.LOG_FILE_LOCATION));
//		nodeList.add(new GPSLocalizer(NodeChannel.POSE));

	}
	
	/***************************************   Getters ********************************/

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
	 * Returns whether the robot is collecting data or not
	 * @return the robot's data collection state
	 */
	public boolean getIsCollectingData() {
		return collectingData;
	}

	/**
	 * Returns a reference to the one instance of the {@link Robot} object.
	 * If no instance exists, a new one is created.
	 * @return a reference to the one instance of the {@link Robot} object
	 */
	public static synchronized  Robot getInstance() {
		if (instance == null) {
			instance = new Robot(!RobobuggyConfigFile.isDataPlayBack());
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
