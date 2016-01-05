package com.roboclub.robobuggy.nodes.baseNodes;

import com.roboclub.robobuggy.ros.Node;

/**
 * Interface node for all BuggyROS nodes used in the Buggy.
 * 
 * @author Zachary Dawson
 *
 */
public interface BuggyNode extends Node {
	
	/**
	 * Called to feed the watchdog of the node. Should be called more often
	 * than the watchdog period defined in {@link BuggyNode}.
	 * This function should only be called by the lowest level implementation
	 * of the {@link BuggyNode} to prevent multiple decorators from
	 * overwritting each other's states.
	 * @param state The current {@link NodeState} of the
	 * {@link BuggyNode} implementation
	 */
	void setNodeState(NodeState state);

}
