package com.roboclub.robobuggy.ros;

public interface Node {
	
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
	
	public void setName(String newName);
	
	public String getName(); 
	
}