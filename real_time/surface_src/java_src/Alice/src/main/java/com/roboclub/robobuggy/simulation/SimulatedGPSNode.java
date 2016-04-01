package com.roboclub.robobuggy.simulation;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.util.Date;

/**
 * A class for simulating the way that the gps system works to allow for offline testing
 * @author Trevor Decker
 *
 */
public class SimulatedGPSNode extends PeriodicNode{
	private Publisher gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
	
	/**
	 * constructor for the simulated gps node
	 */
	public SimulatedGPSNode(int period) {
		super(new BuggyBaseNode(NodeChannel.GPS), period,"simulated_GPS");

		resume();
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void update() {
		 SimulatedBuggy simBuggy = SimulatedBuggy.getInstance();
		 synchronized (this) {
		double xVal = simBuggy.getX();
		double yVal = simBuggy.getY();
		gpsPub.publish(new GpsMeasurement(new Date(), new Date(), yVal, true, xVal, true, 0, 0, 0.0, 0.0));		
		 }
		 }

	@Override
	protected boolean startDecoratorNode() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected boolean shutdownDecoratorNode() {
		// TODO Auto-generated method stub
		return false;
	}
 
	
	
}
