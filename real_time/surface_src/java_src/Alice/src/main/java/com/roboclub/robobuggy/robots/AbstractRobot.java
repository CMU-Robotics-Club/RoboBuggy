package com.roboclub.robobuggy.robots;

import java.util.LinkedList;
import java.util.List;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.RosMaster;
import com.roboclub.robobuggy.ros.Node;

/**
 * A class that encodes the minimum functions need to represent a robot which is much like a ros launch file in normal ros 
 * @author Trevor Decker
 *
 */
public abstract class AbstractRobot implements RosMaster {
	protected List<Node> nodeList;

	
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
	
	/**{@inheritDoc}*/
	@Override
	public List<Node> getNodes() {
		return nodeList;
	}
	
	protected AbstractRobot(){
		nodeList = new LinkedList<>();
	}
	
	/** note all subclasses need to implement a static method that will get the robot */
	

	

}
