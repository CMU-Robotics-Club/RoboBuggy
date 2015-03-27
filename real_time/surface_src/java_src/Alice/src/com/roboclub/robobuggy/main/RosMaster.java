package com.roboclub.robobuggy.main;

import java.util.List;

import com.roboclub.robobuggy.ros.Node;

public interface RosMaster {

	List<Node> getAllSensors();

	// shuts down the robot and all of its child sensors
	boolean shutDown();
}
