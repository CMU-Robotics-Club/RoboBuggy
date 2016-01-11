package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.ros.Message;

public class StateMessage implements Message {
	private Date timestamp;
	private NodeState state;
	
	public StateMessage(NodeState state) {
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

	public NodeState getState() {
		return this.state;
	}
}
