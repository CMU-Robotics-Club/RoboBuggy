package com.roboclub.robobuggy.nodes.sensors;

import java.util.Arrays;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * {@link SerialNode} for reading in IMU data
 * @author Matt Sebek 
 * @author Kevin Brennan
 */
public final class ImuNode extends SerialNode {
	/** Baud rate for serial port */
	private static final int BAUDRATE = 57600;
	// how long the system should wait until a sensor switches to Disconnected
	private static final long SENSOR_TIME_OUT = 5000;
	
	private Publisher msgPub;
	private Publisher statePub;
	
	/**
	 * Creates a new {@link ImuNode}
	 * @param sensor {@link NodeChannel} of IMU
	 * @param portName name of the serial port to read from
	 */
	public ImuNode(NodeChannel sensor, String portName) {
		super(new BuggyBaseNode(sensor), "IMU", portName, BAUDRATE);
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
	
		// TODO state stuff
		statePub.publish(new StateMessage(NodeState.DISCONNECTED));
	}
	
	/**{@inheritDoc}*/
	@Override
	public boolean matchDataSample(byte[] sample) {
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	public int matchDataMinSize() {
		return 0;
	}

	/**{@inheritDoc}*/
	@Override
	public int getBaudRate() {
		return 57600;
	}

	/**{@inheritDoc}*/
	@Override
	public int peel(byte[] buffer, int start, int bytesAvailable) {
		// TODO replace 80 with max message length
		if(bytesAvailable < 30) {
			// Not enough bytes...maybe?
			return 0;
		}
		
		// Check the prefix. was previously #ACG
		if(buffer[start] != '#') {
			return 1;
		}
		if(buffer[start+1] != 'Y') {
			return 1;
		}
		if(buffer[start+2] != 'P') {
			return 1;
		}
		if(buffer[start+3] != 'R') {
			return 1;
		}
		if(buffer[start+4] != '=') {
			return 1;
		}
		
		double[] vals = new double[3];
		String imuRawStr = Arrays.toString(buffer).substring(start+5, bytesAvailable-5); //TODO check +5 -5

		int origLength = imuRawStr.length();
		for (int i = 0; i < 2; i++) {
			// TODO: need less than bytes_availble
			int commaIndex = imuRawStr.indexOf(',');
			try {
				Double d = Double.parseDouble(imuRawStr.substring(0, commaIndex));
				vals[i] = d;
			} catch (NumberFormatException nfe) {
				System.out.println("maligned input; skipping...");
				return 1;
			}
			imuRawStr = imuRawStr.substring(commaIndex+1);	
		}
		
		// The last one, we use the hash as the symbol!
		int hashIndex = imuRawStr.indexOf('#');
		vals[2] = Double.parseDouble(imuRawStr.substring(0, hashIndex));
		imuRawStr = imuRawStr.substring(hashIndex);	
			
		msgPub.publish(new ImuMeasurement(vals[0], vals[1], vals[2]));
		//Feed the watchdog
		setNodeState(NodeState.ON);
		return 4 + (origLength - imuRawStr.length());
	}

	/**
	 * Called to translate a peeled message to a JSON object
	 * @param message {@link String} of the peeled message
	 * @return {@link JSONObject} representing the string
	 */
	@SuppressWarnings("unchecked")
	public static JSONObject translatePeelMessageToJObject(String message) {
		// TODO Auto-generated method stub
		// message has it organized as yaw pitch roll
		JSONObject data = new JSONObject();
		JSONObject params = new JSONObject();
		String[] ypr = message.split(",");
		//0 and 1 will be the name and time
		params.put("yaw", Float.valueOf(ypr[2]));
		params.put("pitch", Float.valueOf(ypr[3]));
		params.put("roll", Float.valueOf(ypr[4]));
		data.put("timestamp", ypr[1]);
		data.put("name", "IMU");
		data.put("params", params);
		return data;
	}
}