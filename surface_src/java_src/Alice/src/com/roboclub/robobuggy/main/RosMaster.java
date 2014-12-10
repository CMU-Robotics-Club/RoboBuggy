package com.roboclub.robobuggy.main;

import java.util.List;

import com.roboclub.robobuggy.sensors.Sensor;

public interface RosMaster {

	List<Sensor> getAllSensors();

	// shuts down the robot and all of its child sensors
	boolean shutDown();
}
