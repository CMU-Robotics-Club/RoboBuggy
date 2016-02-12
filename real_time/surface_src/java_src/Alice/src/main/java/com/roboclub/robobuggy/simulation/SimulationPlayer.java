package com.roboclub.robobuggy.simulation;

import java.util.Date;

import javafx.scene.control.ButtonBar.ButtonData;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

public class SimulationPlayer extends PeriodicNode{
	private Publisher posePub;
	
	
	public SimulationPlayer() {
		super(new BuggyBaseNode(NodeChannel.SIMULATION), 100);
		posePub = new Publisher(NodeChannel.POSE.getMsgPath());
	}
int i = 0;
	@Override
	protected void update() {
		posePub.publish(new GPSPoseMessage(new Date(), i/100.0, i/50.0, i/45.0));
		i = i+1;
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
