package com.roboclub.robobuggy.robots;

import java.io.IOException;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.planners.SweepNode;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.nodes.sensors.*;
import com.roboclub.robobuggy.nodes.planners.WayPointFollowerPlanner;
import com.roboclub.robobuggy.nodes.planners.WayPointUtil;
import com.roboclub.robobuggy.nodes.sensors.CameraNode;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.io.IOException;


/**
 * A robot class for having transistor drive itself
 *
 * @author Trevor Decker
 */
public final class TransistorAuton extends AbstractRobot {
	private static TransistorAuton instance;
	private static final int ARDUINO_BOOTLOADER_TIMEOUT = 2000;

	/**
	 * Returns a reference to the one instance of the {@link} object.
	 * If no instance exists, a new one is created.
	 *
	 * @return a reference to the one instance of the {@link } object
	 */
	public static AbstractRobot getInstance() {
		if (instance == null) {
			instance = new TransistorAuton();
		}
		return instance;
	}

	/**
	 * Constructor for TransistorAuton robot class
	 */
	private TransistorAuton() {
		super();
		try {
			Thread.sleep(ARDUINO_BOOTLOADER_TIMEOUT);
		} catch (InterruptedException e) {
			new RobobuggyLogicNotification("Couldn't wait for bootloader, shutting down", RobobuggyMessageLevel.EXCEPTION);
			shutDown();
		}
		new RobobuggyLogicNotification("Logic Exception Setup properly", RobobuggyMessageLevel.NOTE);
		// Initialize Nodes

		nodeList.add(new HighTrustGPSLocalizer());
		nodeList.add(new GpsNode(NodeChannel.GPS, RobobuggyConfigFile.getComPortGPS()));
//		nodeList.add(new ImuNode(NodeChannel.IMU, RobobuggyConfigFile.getComPortImu()));
		nodeList.add(new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, RobobuggyConfigFile.LOG_FILE_LOCATION,
				NodeChannel.getLoggingChannels()));
		nodeList.add(new RBSMNode(NodeChannel.ENCODER, NodeChannel.STEERING, RobobuggyConfigFile.getComPortRBSM(),
				RobobuggyConfigFile.RBSM_COMMAND_PERIOD));
		nodeList.add(new CameraNode(NodeChannel.PUSHBAR_CAMERA, 100));
		nodeList.add(new HillCrestImuNode());

		try {
			nodeList.add(new WayPointFollowerPlanner(WayPointUtil.createWayPointsFromWaypointList("logs/waypoints/waypoints.txt")));
	}   catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}



	}
}
