package com.roboclub.robobuggy.nodes.baseNodes;

import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.Publisher;

/**
 * Abstract class that serves as the decorator specification in the decorator
 * pattern used to define buggy nodes
 * 
 * @author Zachary Dawson
 *
 */
public abstract class BuggyDecoratorNode implements BuggyNode {
	private String name;
	private BuggyNode node;
	
	/**
	 * Creates a new decorator for the given {@link Node}
	 * @param node {@link Node} to decorate
	 */
	public BuggyDecoratorNode(BuggyNode node,String name) {
		this.node = node;
	}
	
	/**
	 * Set the base node after construction
	 * @param base {@link Node} to make the base node
	 * @return the {@link Node} that was the old base node
	 */
	public final Node setBaseNode(BuggyNode base) {
		Node oldBase = node;
		node = base;
		return oldBase;
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
	
	/**{@inheritDoc}*/
	@Override
	public final void setNodeState(NodeState state) {
		node.setNodeState(state);
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
	
	/**
	 * used for updating the name by which this node is known
	 * @param newName the new name for this node
	 */
	public void setName(String newName){
		name = newName;
	}
	
	/**
	 * gives access to the current value of this nodes name
	 * @return the name of this node
	 */
	public String getName(){
		return name;
	}

}
