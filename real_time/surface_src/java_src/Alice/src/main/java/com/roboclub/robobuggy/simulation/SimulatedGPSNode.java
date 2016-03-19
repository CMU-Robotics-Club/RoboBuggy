package com.roboclub.robobuggy.simulation;

import java.util.Date;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

public class SimulatedGPSNode extends PeriodicNode{
	private Publisher gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
	
	public SimulatedGPSNode() {
		super(new BuggyBaseNode(NodeChannel.GPS), 500);
		
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void update() {
		 SimulatedBuggy simBuggy = SimulatedBuggy.GetInstance();
		double xVal = simBuggy.getX();
		double yVal = simBuggy.getY();
		gpsPub.publish(new GpsMeasurement(new Date(), new Date(), xVal, true, yVal, true, 0, 0, 0.0, 0.0));		
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
