package com.roboclub.robobuggy.main;

import java.util.List;
import com.roboclub.robobuggy.ros.Node;

/**
 * Interface to dictate the behaviors of a BuggyROS master
 */
public interface RosMaster {

	/**
	 * Returns a list of the {@link Node}s in the system
	 * @return a list of the {@link Node}s in the system
	 */
	List<Node> getNodes();

	/**
	 * Starts the {@link RosMaster} and all its {@link Node}s safely
	 * @return true iff the {@link RosMaster} starts successfully
	 */
	boolean startNodes();
	
	/**
	 * Shuts down the {@link RosMaster} safely
	 * @return true iff the {@link RosMaster} shut down successfully
	 */
	boolean shutDown();
}
