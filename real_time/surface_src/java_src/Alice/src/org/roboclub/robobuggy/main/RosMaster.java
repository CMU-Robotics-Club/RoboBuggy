package org.roboclub.robobuggy.main;

import java.util.List;
import org.roboclub.robobuggy.ros.Node;

/**
 * @author Matt Sebeck
 * @version 0.0
 * TODO document 
 *
 */
public interface RosMaster {

	List<Node> getAllSensors();

	/**
	 *  shuts down the robot and all of its child sensors
	 * @return
	 */
	boolean shutDown();
}
