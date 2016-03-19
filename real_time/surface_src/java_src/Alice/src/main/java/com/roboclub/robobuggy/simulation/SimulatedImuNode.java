package com.roboclub.robobuggy.simulation;

import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

public class SimulatedImuNode extends PeriodicNode{
	private Publisher imuPub = new Publisher(NodeChannel.IMU.getMsgPath());
	private Publisher imuMagPub = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());
	protected SimulatedImuNode(BuggyNode base, int period) {
		super(base, period);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void update() {
		SimulatedBuggy simBuggy = SimulatedBuggy.GetInstance();
		imuPub.publish(new ImuMeasurement(simBuggy.getTh(), 0.0, 0.0));
		imuMagPub.publish(new MagneticMeasurement(0.0, 0.0, simBuggy.getTh()));
		
		// TODO Auto-generated method stub
		
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
