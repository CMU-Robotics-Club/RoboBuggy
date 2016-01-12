package com.roboclub.robobuggy.main;

import java.util.LinkedList;
import java.util.List;

import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * 
 * The user interface controls what gui is  displayed to the user and how they can interact with system 
 *
 */
public class Robot implements RosMaster {

	/************************************** Sets all internal private variables *************************/
	private static final int COMMAND_PERIOD = 1000;
	private static Robot instance;
	private boolean autonomous;
	private List<Node> nodeList;
	
	/************************************* Set of all public functions ********************************/

	@Override
	public boolean shutDown() {
		return nodeList.stream().map(n -> n.shutdown()).reduce(true, (a,b) -> a&&b);
	}
	
	public boolean startNode() {
		return nodeList.stream().map(n -> n.startNode()).reduce(true, (a,b) -> a&&b);
	}
	
	/************************************* Set of all internal private functions ************************/
	private Robot() {
		System.out.println("Starting Robot");
		autonomous = RobobuggyConfigFile.AUTONOMUS_DEFAULT;
		nodeList = new LinkedList<>();
		RobobuggyLogicException.setupLogicException(NodeChannel.LOGIC_EXCEPTION);
		new RobobuggyLogicException("Logic Exception Setup properly" ,  RobobuggyMessageLevel.NOTE);
		
		// Initialize Nodes
		nodeList.add(new GpsNode(NodeChannel.GPS, RobobuggyConfigFile.COM_PORT_GPS_INTEGRATED));
		nodeList.add(new ImuNode(NodeChannel.IMU, RobobuggyConfigFile.COM_PORT_IMU));
		nodeList.add(new RBSMNode(NodeChannel.ENCODER, NodeChannel.STEERING,
				RobobuggyConfigFile.COM_PORT_ENCODER, COMMAND_PERIOD));
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

	@Override
	public List<Node> getNodes() {
		return nodeList;
	}
	
	/***************************************  setters ********************************/
}
