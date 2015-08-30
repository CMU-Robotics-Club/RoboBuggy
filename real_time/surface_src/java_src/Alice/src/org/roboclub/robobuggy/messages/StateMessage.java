package org.roboclub.robobuggy.messages;

import java.util.Date;

import org.roboclub.robobuggy.ros.Message;
import org.roboclub.robobuggy.sensors.SensorState;

/**
 * @author Matt Sebek
 * @version 0.0
 *
 */
public class StateMessage implements Message {
	private Date timestamp;
	private SensorState state;
	
	public StateMessage(SensorState state) {
		this.timestamp = new Date();
		this.state = state;
	}
	
	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Message fromLogString(String str) {
		// TODO Auto-generated method stub
		return null;
	}

	public SensorState getState() {
		return this.state;
	}
}
