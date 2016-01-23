package com.roboclub.robobuggy.messages;

import java.util.Date;

import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.ros.Message;

/**
 * Message used for passing {@link NodeState} information over BuggyROS
 */
public class StateMessage implements Message {
	private Date timestamp;
	private NodeState state;
	
	/**
	 * Construct a new {@link StateMessage} with time now
	 * @param state {@link NodeState} to transmit
	 */
	public StateMessage(NodeState state) {
		this.timestamp = new Date();
		this.state = state;
	}

	/**
	 * Get the timestamp of the state message
	 * @return the timestamp of this message
	 */
	public Date getTimestamp(){
		return new Date(timestamp.getTime());
	}
	
	/**{@inheritDoc}*/
	@Override
	public String toLogString() {
		// TODO Auto-generated method stub
		return null;
	}

	/**{@inheritDoc}*/
	@Override
	public Message fromLogString(String str) {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * Returns the {@link NodeState} sent in the {@link StateMessage}
	 * @return the {@link NodeState} sent in the {@link StateMessage}
	 */
	public NodeState getState() {
		return this.state;
	}
}
