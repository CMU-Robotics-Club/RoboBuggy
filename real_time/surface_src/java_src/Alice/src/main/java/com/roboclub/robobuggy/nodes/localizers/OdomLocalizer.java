package com.roboclub.robobuggy.nodes.localizers;

import java.util.Date;

import com.roboclub.robobuggy.map.So2Pose;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.PoseMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.LocTuple;


/**
 * 
 * @author Trevor Decker
 *
 *  This node produces a position estimate with full trust based only on input from the front wheel
 *   encoder and the front wheel commanded angle 
 *
 */
public class OdomLocalizer  extends PeriodicNode{
	private Publisher posePub;
	private So2Pose pose = new So2Pose(0.0, 0.0, 0.0);
	
	private double mostRecentEncoder = 0;
	private double secondOldestEncoder = 0;
	private double mostRecentAngle = 0;
	private double secondOldestAngle = 0;
	
	
	public OdomLocalizer() {
		super(new  BuggyBaseNode(NodeChannel.POSE), 1000);
		//Initialize subscriber to encoder measurements
		new Subscriber(NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
		@Override
		public void actionPerformed(String topicName, Message m) {
			EncoderMeasurement encM = (EncoderMeasurement)m;
			//TODO add locks 
			secondOldestEncoder = mostRecentEncoder;
			mostRecentEncoder = encM.getDistance();
			}
		});
			
		//Initialize subscriber for what steering angle low level is currently at
		new Subscriber(NodeChannel.STEERING_COMMANDED.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				SteeringMeasurement steerMeasur =  (SteeringMeasurement)m;
				//TODO add locks
				secondOldestAngle = mostRecentAngle;
				mostRecentAngle = steerMeasur.getAngle();				
			}
		});
		
			
			
			
	}


	@Override
	protected void update() {
		double deltaAngle = mostRecentAngle - secondOldestAngle;
		double deltaEncoder = mostRecentEncoder - secondOldestEncoder;
		So2Pose deltaPose = new So2Pose(deltaEncoder, 0.0, deltaAngle);
		pose = deltaPose.mult(pose);
		posePub.publish(new PoseMessage(new Date(), pose.getX(), pose.getY(), pose.getOrintation()));
		
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
