package com.roboclub.robobuggy.simulation;

import com.roboclub.robobuggy.map.So2Pose;
import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.DriveControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * 
 * This node listens to commands from  topics and produces messages similar to what the RBSM node would do
 * based on a model of how the buggy behaves 
 * 
 * @author Trevor Decker
 *
 */
public class SimulatedRBSMNode extends PeriodicNode{
	//for determining the encoder and commanded angle based on the 
	//transform between the current pose and the last pose
	private So2Pose lastPose = new So2Pose(0,0,0);
	private double encoderDistance = 0;
	private Publisher messagePubEnc;
	private Publisher messagePubControllerSteering;
	private int commandedAngle = 0;
	private boolean commandedBrakeEngaged = true;

	
	/**
	 * The constructor for the simulatedRBSMNode
	 */
	 public SimulatedRBSMNode() {
		super(new BuggyBaseNode(NodeChannel.ENCODER), 1000,"simulated_rbsm_node");
		SimulatedBuggy simBuggy = SimulatedBuggy.getInstance();
		// TODO Auto-generated constructor stub
		messagePubEnc = new Publisher(NodeChannel.ENCODER.getMsgPath());
		messagePubControllerSteering = new Publisher(NodeChannel.STEERING_COMMANDED.getMsgPath());
		
		//changes the angle that we are attempting to drive at based on the commanded angle messages we are getting 
		//Initialize subscribers to commanded angle and brakes state
		new Subscriber(NodeChannel.DRIVE_CTRL.getMsgPath(),
				new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				commandedAngle = ((DriveControlMessage)m).getAngleInt();
			}
		});
		
		new Subscriber(NodeChannel.BRAKE_CTRL.getMsgPath(),
				new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				commandedBrakeEngaged = ((BrakeControlMessage)m).isBrakeEngaged();
				simBuggy.setBrakesDown(commandedBrakeEngaged);
			}
		});
		resume();//needed to start the node
	}

	@Override
	protected void  update() {
		SimulatedBuggy simBuggy = SimulatedBuggy.getInstance();
		if(lastPose == null){
			lastPose = new So2Pose(simBuggy.getX(), simBuggy.getY(), simBuggy.getTh());
		}
		//get an updated representation of the buggy current pose	
		So2Pose newPose = new So2Pose(simBuggy.getX(),simBuggy.getY(), simBuggy.getTh());
		//find the difference between the last observed pose and this pose
		So2Pose dPose = newPose.mult(lastPose.inverse());
		lastPose = newPose;

		//use that information to fill out messages to be published 
		//the distance between the two is the amount which the encoder would be incremented
		double dx = dPose.getX();
		double dy = dPose.getY();
		double d = Math.sqrt(dx*dx + dy*dy);
		encoderDistance = encoderDistance + d;
		
		messagePubEnc.publish(new EncoderMeasurement(encoderDistance, 0.0)); //TODO calculate the actual velocity 
		double potAngle = Math.atan2(dy, dx);
		messagePubControllerSteering.publish(new SteeringMeasurement(potAngle));

		int outputAngle = commandedAngle;
		//update the commanded angle 
		if(outputAngle > 1000) {
			outputAngle = 1000;
		}
		else if (outputAngle < -1000) {
			outputAngle = -1000;
		}
		
		
		//TODO set velocity so that the buggy will try and steer towards this angle instead of jumping directly to it 
		simBuggy.setWheelTh((double)outputAngle/1000.0); 
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
