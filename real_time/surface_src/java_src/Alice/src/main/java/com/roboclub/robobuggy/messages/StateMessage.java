package com.roboclub.robobuggy.messages;

import com.roboclub.robobuggy.nodes.baseNodes.NodeState;

import java.util.Date;

/**
 * Message used for passing {@link NodeState} information over BuggyROS
 */
public class StateMessage extends BaseMessage {
	public static final String VERSION_ID = "state";
	private NodeState state;

	/**
	 * Construct a new {@link StateMessage} with time now
	 * @param state {@link NodeState} to transmit
	 */
	public StateMessage(NodeState state) {
		this.timestamp = new Date().getTime();
		this.state = state;
	}

	/**
	 * Returns the {@link NodeState} sent in the {@link StateMessage}
	 * @return the {@link NodeState} sent in the {@link StateMessage}
	 */
	public NodeState getState() {
		return this.state;
	}
}
