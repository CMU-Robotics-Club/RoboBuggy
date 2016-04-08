package com.roboclub.robobuggy.simulation;

import com.roboclub.robobuggy.messages.IMUAngularPositionMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
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
	private Publisher imuRotPub = new Publisher(NodeChannel.IMU_ANG_POS.getMsgPath());
	private SimulatedBuggy simBuggy = SimulatedBuggy.getInstance();

	/**
	 * Constructor for the simulated imu node
	 * @param period how many milliseconds between new simulated imu messages
	 */
	public SimulatedImuNode(int period) {
		super(new BuggyBaseNode(NodeChannel.IMU), period,"simulated_imu_node");
		simBuggy = SimulatedBuggy.getInstance();
		// TODO Auto-generated constructor stub
		resume();
	}

	@Override
	protected void update() {
		imuPub.publish(new ImuMeasurement(simBuggy.getTh(), 0.0, 0.0));
		double th = Math.toRadians(simBuggy.getTh());
		double[][] rotMat = {{Math.cos(th),Math.sin(th), 0},
				             {-Math.sin(th),Math.cos(th	),0},
				             {0,0,1}};
		imuRotPub.publish(new IMUAngularPositionMessage(rotMat));
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
