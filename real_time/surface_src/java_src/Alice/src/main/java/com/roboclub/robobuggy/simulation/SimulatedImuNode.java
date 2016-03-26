package com.roboclub.robobuggy.simulation;

import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

/**
 * A simulated version of the imu for off board testing and gaining a better understanding of how the system works
 * @author Trevor Decker
 *
 */
public class SimulatedImuNode extends PeriodicNode{
	private Publisher imuPub = new Publisher(NodeChannel.IMU.getMsgPath());
	private Publisher imuMagPub = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());
	private static SimulatedBuggy simBuggy = SimulatedBuggy.getInstance();

	protected SimulatedImuNode(BuggyNode base, int period) {
		super(base, period);
		simBuggy = SimulatedBuggy.getInstance();
		// TODO Auto-generated constructor stub
		resume();
	}

	@Override
	protected void update() {
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
