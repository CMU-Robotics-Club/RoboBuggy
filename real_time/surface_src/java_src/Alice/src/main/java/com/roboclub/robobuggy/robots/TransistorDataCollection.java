package com.roboclub.robobuggy.robots;

import java.util.LinkedList;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.nodes.localizers.HighTrustGPSLocalizer;
import com.roboclub.robobuggy.nodes.planners.SweepNode;
import com.roboclub.robobuggy.nodes.sensors.GpsNode;
import com.roboclub.robobuggy.nodes.sensors.ImuNode;
import com.roboclub.robobuggy.nodes.sensors.LoggingNode;
import com.roboclub.robobuggy.nodes.sensors.RBSMNode;
import com.roboclub.robobuggy.ros.NodeChannel;

public class TransistorDataCollection extends AbstractRobot {

	private static final int ARDUINO_BOOTLOADER_TIMEOUT = 2000;
	
	/**
	 * Returns a reference to the one instance of the {@link Robot} object.
	 * If no instance exists, a new one is created.
	 * @return a reference to the one instance of the {@link Robot} object
	 */
	public static AbstractRobot getInstance() {
		if (instance == null) {
			instance =  new TransistorDataCollection();
		}
		return instance;
	}
	
	private TransistorDataCollection(){
		super();
	System.out.println("Starting Robot");
	try {
		Thread.sleep(ARDUINO_BOOTLOADER_TIMEOUT);
	} catch (InterruptedException e) {
		new RobobuggyLogicNotification("Couldn't wait for bootloader, shutting down", RobobuggyMessageLevel.EXCEPTION);
		shutDown();
	}
	new RobobuggyLogicNotification("Logic Exception Setup properly" ,  RobobuggyMessageLevel.NOTE);
	// Initialize Nodes
		nodeList.add(new ImuNode(NodeChannel.IMU, RobobuggyConfigFile.COM_PORT_IMU));
		nodeList.add(new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, RobobuggyConfigFile.LOG_FILE_LOCATION,
				NodeChannel.getLoggingChannels()));
		nodeList.add(new HighTrustGPSLocalizer());

}
}

