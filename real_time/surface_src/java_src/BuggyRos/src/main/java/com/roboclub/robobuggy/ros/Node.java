package com.roboclub.robobuggy.ros;

public interface Node {
	String name = "Node";
	
	/**
	 * Called to start running the code of the respective node
	 * @return true iff the node is successfully started
	 */
	boolean startNode();
	
	/**
	 * Safely shuts down the node object
	 * @return true iff successfully shut down
	 */
	boolean shutdown();
	
	default String getName() {
		// TODO Auto-generated method stub
		return Node.name;
	}
	
}