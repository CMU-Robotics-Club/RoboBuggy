package com.roboclub.robobuggy.nodes.baseNodes;

import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Base implementation of a Buggy {@link BuggyNode} to be decorated
 * with {@link BuggyDecoratorNode}s according to the decorator pattern.
 * The {@link BuggyBaseNode} implements the functionality for health/status
 * monitoring.
 * 
 * @author Zachary Dawson
 *
 */
public class BuggyBaseNode implements BuggyNode {
	private String name;
	private static final long WATCHDOG_PERIOD = 2000;
	
	private Publisher statePub;
	private NodeState state;
	private Long lastUpdate;
	
	/**
	 * Construct a new {@link BuggyBaseNode}
	 * @param nodeChannel {@link NodeChannel} for the node being created
	 */
	public BuggyBaseNode(NodeChannel nodeChannel) {
		statePub = new Publisher(nodeChannel.getStatePath());
		state = NodeState.NOT_IN_USE;
		lastUpdate = 0L;
		TimerTask timerTask = new UpdateTask();
        Timer timer = new Timer(true);
        timer.scheduleAtFixedRate(timerTask, 0, WATCHDOG_PERIOD);
	}
	
	/**{@inheritDoc}*/
	@Override
	public boolean startNode() {
		//Set the state to be on and record the time
		state = NodeState.ON;
		lastUpdate = System.currentTimeMillis();
		//Return true as the base node can always be started
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	public boolean shutdown() {
		//Set the state to not in use
		state = NodeState.NOT_IN_USE;
		//Return true as the base node can always be shut down
		return true;
	}

	/**{@inheritDoc}*/
	@Override
	public void setNodeState(NodeState state) {
		this.state = state;
		lastUpdate = System.currentTimeMillis();
	}
	
	/**
     * Task used to call the update method
     */
	private class UpdateTask extends TimerTask{
	
	    @Override
	    public void run() {
	    	if(System.currentTimeMillis()-lastUpdate > WATCHDOG_PERIOD &&
	    			state == NodeState.ON) {
	    		state = NodeState.WATCHDOG_DEAD;
	    	}
	    	statePub.publish(new StateMessage(state));
	    }
	}
	
	/**
	 * used for updating the name by which this node is known
	 * @param newName the new name for this node
	 */
	public void setName(String newName){
		name = newName;
	}
	
	/**
	 * gives access to the current value of this nodes name
	 * @return the name of the node
	 */
	public String getName(){
		return name;
	}

}
