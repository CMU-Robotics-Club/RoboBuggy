package com.roboclub.robobuggy.nodes;

import com.roboclub.robobuggy.ros.Node;

/**
 * Base implementation of a Buggy {@link Node} to be decorated with
 * BuggyDecoratorNodes according to the decorator pattern
 * 
 * @author Zachary Dawson
 *
 */
public class BuggyBaseNode implements Node {

	/**{@inheritDoc}*/
	@Override
	public boolean startNode() {
		//Return true as the base node can always be started
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	public boolean shutdown() {
		//Return true as the base node can always be shut down
		return true;
	}

}
