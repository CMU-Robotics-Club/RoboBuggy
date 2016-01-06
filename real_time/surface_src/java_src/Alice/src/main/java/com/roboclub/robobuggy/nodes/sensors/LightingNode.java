package com.roboclub.robobuggy.nodes.sensors;

import com.orsoncharts.util.json.JSONObject;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;

/**
 * {@link SerialNode} for reading in lighting data
 * @author Matt Sebek 
 * @author Kevin Brennan
 *
 * DESCRIPTION: TODO
 */

public class LightingNode extends SerialNode {
	// how long the system should wait until a sensor switches to Disconnected
	private static final long SENSOR_TIME_OUT = 5000;

	private static final int BAUD_RATE = 9600;
	
	private Publisher msgPub;
	private Publisher statePub;
	
	/**
	 * Creates a new {@link LightingNode}
	 * @param sensor {@link NodeChannel} of lighting unit
	 * @param portName name of the serial port to read from
	 */
	public LightingNode(NodeChannel sensor, String portName) {
		super(new BuggyBaseNode(sensor), "Lighting", portName, BAUD_RATE);
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
	
		// TODO state stuff
		//statePub.publish(new StateMessage(this.currState));
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
	public int baudRate() {
		return BAUD_RATE;
	}

	/**{@inheritDoc}*/
	@Override
	public int peel(byte[] buffer, int start, int bytesAvailable) {
		// TODO replace 80 with max message length
		if(bytesAvailable < 80) {
			// Not enough bytes...maybe?
			return 0;
		}
		
		// Check the prefix.
		if(buffer[start] != '#') {
			return 1;
		}
		if(buffer[start+1] != 'A') {
			return 1;
		}
		if(buffer[start+2] != 'C') {
			return 1;
		}
		if(buffer[start+3] != 'G') {
			return 1;
		}
		if(buffer[start+4] != '=') {
			return 1;
		}
		double[] vals = new double[9];
		String b = new String(buffer, start+5, bytesAvailable-5);
		int origLength = b.length();
		for (int i = 0; i < 8; i++) {
			// TODO: need less than bytes_availble
			int commaIndex = b.indexOf(',');
			try {
				vals[i] = Double.parseDouble(b.substring(0, commaIndex)); 
			} catch (NumberFormatException nfe) {
				System.out.println("maligned input; skipping...");
				return 1;
			}
			b = b.substring(commaIndex+1);	
		}
		
		// The last one, we use the hash as the symbol!
		int hashIndex = b.indexOf('#');
		vals[8] = Double.parseDouble(b.substring(0, hashIndex));
		b = b.substring(hashIndex);	
			
//		msgPub.publish(new ImuMeasurement(vals[0], vals[1],vals[2], 
//				vals[3], vals[4], vals[5], vals[6], vals[7], vals[8]));
		//Feed the watchdog
		setNodeState(NodeState.ON);
		return 4 + (origLength - b.length());
	}

	/**
	 * Called to translate a peeled message to a JSON object
	 * @param message {@link String} of the peeled message
	 * @return {@link JSONObject} representing the string
	 */
	public static JSONObject translatePeelMessageToJObject(String message) {
		// TODO Auto-generated method stub
		return null;
	}

}