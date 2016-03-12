package com.roboclub.robobuggy.robots;

import java.util.LinkedList;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.planners.SweepNode;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.NodeChannel;

public class TransistorAuton extends AbstractRobot{

	private static final int ARDUINO_BOOTLOADER_TIMEOUT = 2000;
	
	/**
	 * Returns a reference to the one instance of the {@link Robot} object.
	 * If no instance exists, a new one is created.
	 * @return a reference to the one instance of the {@link Robot} object
	 */
	public static AbstractRobot getInstance() {
		if (instance == null) {
			instance =  new TransistorAuton();
		}
		return instance;
	}
	
	private TransistorAuton(){
		super();
	System.out.println("Starting Robot");
	try {
		Thread.sleep(ARDUINO_BOOTLOADER_TIMEOUT);
	} catch (InterruptedException e) {
		new RobobuggyLogicNotification("Couldn't wait for bootloader, shutting down", RobobuggyMessageLevel.EXCEPTION);
		shutDown();
	}
	new RobobuggyLogicNotification("Logic Exception Setup properly" ,  RobobuggyMessageLevel.NOTE);
	// Initialize Nodes
	
//	nodeList.add(new SimulationPlayer());
	//nodeList.add(new OdomLocalizer());
	nodeList.add(new HighTrustGPSLocalizer());
	
//	nodeList.add(new OdomLocalizer());
	
	//this subset of nodes is only added when we are in online mode 
		nodeList.add(new GpsNode(NodeChannel.GPS, RobobuggyConfigFile.COM_PORT_GPS));
		nodeList.add(new ImuNode(NodeChannel.IMU, RobobuggyConfigFile.COM_PORT_IMU));
		nodeList.add(new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, RobobuggyConfigFile.LOG_FILE_LOCATION,
				NodeChannel.getLoggingChannels()));
		nodeList.add(new RBSMNode(NodeChannel.ENCODER, NodeChannel.STEERING, RobobuggyConfigFile.COM_PORT_RBSM,
				RobobuggyConfigFile.RBSM_COMMAND_PERIOD));
	

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

//	nodeList.add(new CameraNode(NodeChannel.PUSHBAR_CAMERA, 100));
//	nodeList.add(new GPSTrackPlannerNode(NodeChannel.BRAKE_CTRL,RobobuggyConfigFile.LOG_FILE_LOCATION));
//	nodeList.add(new GPSLocalizer(NodeChannel.POSE));

}
}
