package com.roboclub.robobuggy.nodes.planners;

import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyDecoratorNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

import java.util.Date;

/**
 * Abstract class used to represent all {@link BuggyNode}s used as path 
 * planners.
 * 
 * @author Zachary Dawson
 *
 */
public abstract class PathPlannerNode extends BuggyDecoratorNode {

	private Publisher steeringCommandPub;
	private Publisher brakingCommandPub;
	
	/**
	 * Construct a new {@link PathPlannerNode}
	 * @param channel {@link NodeChannel} on which to broadcast status
	 *  information about the node
	 */
	public PathPlannerNode(NodeChannel channel) {
		super(new BuggyBaseNode(channel),"pathPlannerNode");
		steeringCommandPub = new Publisher(NodeChannel.DRIVE_CTRL.getMsgPath());
		brakingCommandPub = new Publisher(NodeChannel.BRAKE_CTRL.getMsgPath());
	}
	
	/**{@inheritDoc}*/
	@Override
	protected final boolean startDecoratorNode() {
		//Initialize subscribers to pose estimations
		new Subscriber(NodeChannel.POSE.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				updatePositionEstimate((GPSPoseMessage)m);
				steeringCommandPub.publish(new DriveControlMessage(new Date(),
						getCommandedSteeringAngle()));
				brakingCommandPub.publish(new BrakeControlMessage(new Date(),
						getDeployBrakeValue()));
			}
		});
		return true;
	}
	
	/**{@inheritDoc}*/
	@Override
	protected boolean shutdownDecoratorNode() {
		//Always shuts down properly
		return true;
	}
	
	/**
	 * Called whenever there is an update to the robots pose. This method 
	 * updates the planner's notion of the robot pose, causing a re-planning 
	 * of the path if needed. 
	 * @param m {@link PoseMessage} containing the new pose of the robot
	 */
	protected abstract void updatePositionEstimate(GPSPoseMessage m);
	
	/**
	 * Returns the steering angle to which the {@link PathPlanner} thinks the 
	 * buggy's steering should be commanded to follow the desired path.
	 * @return desired commanded steering angle
	 */
	protected abstract double getCommandedSteeringAngle();
	
	/**
	 * Returns the brake value to which the {@link PathPlanner} thinks the 
	 * buggy's brakes should be commanded.
	 * @return desired commanded brake value (true is deployed)
	 */
	protected abstract boolean getDeployBrakeValue();
}
