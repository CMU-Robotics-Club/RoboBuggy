package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.nodes.baseNodes.SerialNode;
import com.roboclub.robobuggy.ros.NodeChannel;

import java.io.UnsupportedEncodingException;

/**
 * {@link SerialNode} for reading in lighting data
 * @author Matt Sebek 
 * @author Kevin Brennan
 *
 * DESCRIPTION: TODO
 */

public class LightingNode extends SerialNode {

	private static final int BAUD_RATE = 9600;
	

	
	/**
	 * Creates a new {@link LightingNode}
	 * @param sensor {@link NodeChannel} of lighting unit
	 * @param portName name of the serial port to read from
	 */
	public LightingNode(NodeChannel sensor, String portName) {
		super(new BuggyBaseNode(sensor), "Lighting", portName, BAUD_RATE);

	
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
	public int peel(byte[] buffer, int start, int bytesAvailable) {
		// TODO replace 80 with max message length
		if(bytesAvailable < 80) {
			// Not enough bytes...maybe?
			return 0;
		}

		String lightingRawStr;
		try {
			lightingRawStr = new String(buffer, start+5, bytesAvailable-5, "UTF-8");
		} catch (UnsupportedEncodingException e) {
			return 1;
		}//TODO check +5 -5

		int origLength = lightingRawStr.length();

		//Feed the watchdog
		setNodeState(NodeState.ON);
		return 4 + (origLength - lightingRawStr.length());
	}



}