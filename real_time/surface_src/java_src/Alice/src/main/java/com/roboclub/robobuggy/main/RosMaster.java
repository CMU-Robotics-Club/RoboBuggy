package com.roboclub.robobuggy.main;

import java.util.List;
import com.roboclub.robobuggy.ros.Node;

public interface RosMaster {

	/**
	 * Returns a list of the {@link Node}s in the system
	 * @return a list of the {@link Node}s in the system
	 */
	List<Node> getNodes();

	/**
	 * Shuts down the {@link RosMaster} safely
	 * @return true iff the {@link RosMaster} shut down sucessfuly
	 */
	boolean shutDown();
}
