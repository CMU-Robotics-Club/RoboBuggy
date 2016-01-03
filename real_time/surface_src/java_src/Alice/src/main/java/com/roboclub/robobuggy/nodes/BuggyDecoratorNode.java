package com.roboclub.robobuggy.nodes;

import com.roboclub.robobuggy.ros.Node;

/**
 * Abstract class that serves as the decorator specification in the decorator
 * pattern used to define buggy nodes
 * 
 * @author Zachary Dawson
 *
 */
public abstract class BuggyDecoratorNode implements Node {
	
	private final Node node;
	
	/**
	 * Creates a new decorator for the given {@link Node}
	 * @param node {@link Node} to decorate
	 */
	public BuggyDecoratorNode(Node node) {
		this.node = node;
	}

	/**Starts the underlying {@link Node} and its decorator (in that order)
	  *{@inheritDoc}*/
	@Override
	public final boolean startNode() {
		boolean nodeStartResult = node.startNode();
		return nodeStartResult && startDecoratorNode();
	}

	/**Shuts down the decorator and its underlying {@link Node} (in that order)
	  *{@inheritDoc}*/
	@Override
	public final boolean shutdown() {
		boolean nodeShutdownResult = node.shutdown();
		return shutdownDecoratorNode() && nodeShutdownResult;
	}
	
	/**
	 * Starts the decorator of the {@link Node}
	 * Called after the {@link Node} being decorated is started
	 * @return true iff the decorator is started successfully
	 */
	protected abstract boolean startDecoratorNode();
	
	/**
	 * Shuts down the decorator of the {@link Node}
	 * Called immediately before the {@link Node} being decorated is shut down
	 * @return true iff the decorator is shut down successfully
	 */
	protected abstract boolean shutdownDecoratorNode();

}
