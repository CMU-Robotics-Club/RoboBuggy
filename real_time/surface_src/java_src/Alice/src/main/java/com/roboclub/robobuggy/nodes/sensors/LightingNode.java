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
 * @author Matt Sebek 
 * @author Kevin Brennan
 *
 * DESCRIPTION: TODO
 */

public class LightingNode extends SerialNode {
	// how long the system should wait until a sensor switches to Disconnected
	private static final long SENSOR_TIME_OUT = 5000;

	private static final int BAUD_RATE = 9600;
	
	public Publisher msgPub;
	public Publisher statePub;
	
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
	public int getBaudRate() {
		return BAUD_RATE;
	}

	/**{@inheritDoc}*/
	@Override
	public int peel(byte[] buffer, int start, int bytes_available) {
		// TODO replace 80 with max message length
		if(bytes_available < 80) {
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
		String b = new String(buffer, start+5, bytes_available-5);
		int orig_length = b.length();
		for (int i = 0; i < 8; i++) {
			// TODO: need less than bytes_availble
			int comma_index = b.indexOf(',');
			try {
				vals[i] = Double.parseDouble(b.substring(0, comma_index)); 
			} catch (NumberFormatException nfe) {
				System.out.println("maligned input; skipping...");
				return 1;
			}
			b = b.substring(comma_index+1);	
		}
		
		// The last one, we use the hash as the symbol!
		int hash_index = b.indexOf('#');
		vals[8] = Double.parseDouble(b.substring(0, hash_index));
		b = b.substring(hash_index);	
			
//		msgPub.publish(new ImuMeasurement(vals[0], vals[1],vals[2], 
//				vals[3], vals[4], vals[5], vals[6], vals[7], vals[8]));
		//Feed the watchdog
		setNodeState(NodeState.ON);
		return 4 + (orig_length - b.length());
	}

	
	public static JSONObject translatePeelMessageToJObject(String message) {
		// TODO Auto-generated method stub
		return null;
	}

}