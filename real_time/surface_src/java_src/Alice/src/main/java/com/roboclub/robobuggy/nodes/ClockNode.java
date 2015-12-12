package com.roboclub.robobuggy.nodes;

import java.util.Date;

import com.roboclub.robobuggy.messages.TimeMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;

// is for timing work, so that the system can keep track of time properly in play back
// This is especial important for faster or slower then real time playback speed

//This node may be replaceable by looking at the time stamps on all the other messages 
public class ClockNode extends PeriodicNode{

	private Publisher msgPub;

	public ClockNode(SensorChannel sensor) {
		super(100); //updates in 1/10th of a second
		
		msgPub = new Publisher(sensor.getMsgPath());
	}

	@Override
	public boolean shutdown() {
		return false;
	}

	@Override
	protected void update() {
		msgPub.publish(new TimeMessage(new Date()));
		//TODO send time messaage 
		
	}

}
