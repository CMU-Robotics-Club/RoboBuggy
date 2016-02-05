package com.roboclub.robobuggy.simulation;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Date;

import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage.LoggingMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.utilities.RobobuggyDateFormatter;

/**
 * Class used for playing back old log files. It does this by reading BuggyROS
 * messages stored in the log file and reinjecting them into the BuggyROS network.
 */
public class SensorPlayer implements Runnable {

	private String path;

	private Publisher imuPub;
	private Publisher gpsPub;
	private Publisher encoderPub;
	//private Publisher brakePub; TODO: future expansion
	private Publisher steeringPub;
	private Publisher loggingButtonPub;


	/**
	 * Construct a new {@link SensorPlayer} object
	 * @param filePath {@link String} of the name and location of the log file
	 */
	public SensorPlayer(String filePath) {

		imuPub = new Publisher(NodeChannel.IMU.getMsgPath());
		gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
		encoderPub = new Publisher(NodeChannel.ENCODER.getMsgPath());
	//	brakePub = new Publisher(NodeChannel.BRAKE.getMsgPath()); TODO: future expansion
		steeringPub = new Publisher(NodeChannel.STEERING.getMsgPath());
		loggingButtonPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());

		System.out.println("initializing the SensorPlayer");

		path = filePath;
		File f = new File(path);
		if(!f.exists()) {
			new RobobuggyLogicNotification("File doesn't exist!", RobobuggyMessageLevel.EXCEPTION);
		}
	}


	@Override
	public void run() {
		// TODO Auto-generated method stub

	}
}
