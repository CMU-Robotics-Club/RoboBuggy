package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.map.So2Pose;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

import java.util.Date;


/**
 * 
 * @author Trevor Decker
 *
 *  This node produces a position estimate with full trust based only on input from the front wheel
 *   encoder and the front wheel commanded angle 
 *
 */
public class OdomLocalizer  extends PeriodicNode{
	private So2Pose pose;
	private Publisher posePub = new Publisher(NodeChannel.POSE.getMsgPath());

	private double currentEncoderVal = 0;
	private double previousEncoderVal = 0;
	private double currentAngle = 0;
	private double previousAngle = 0;


	/**
	 * initializes a new odomlocalizer
	 */
	public OdomLocalizer() {
		super(new  BuggyBaseNode(NodeChannel.POSE), 100,"odomLocalizer");
		
		
		//Initialize subscriber to encoder measurements
		new Subscriber(NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
		@Override
		public void actionPerformed(String topicName, Message m) {
			EncoderMeasurement encM = (EncoderMeasurement)m;
			//TODO add locks 
			double dist = encM.getDistance();
				previousEncoderVal = currentEncoderVal;
				currentEncoderVal = dist;
			if (pose == null) {
				pose = new So2Pose(0.0, 0.0, 0.0);
			}
			double deltaAngle = currentAngle - previousAngle;
			double deltaEncoder = currentEncoderVal - previousEncoderVal;
			So2Pose deltaPose = new So2Pose(deltaEncoder, 0.0, deltaAngle);
			pose = pose.mult(deltaPose);
			posePub.publish(new GPSPoseMessage(new Date(), pose.getX(), pose.getY(), pose.getOrientation()));
			previousAngle = currentAngle;
			
						
			}
		});
			
		//Initialize subscriber for what steering angle low level is currently at
		new Subscriber(NodeChannel.STEERING.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				SteeringMeasurement steerMeasur =  (SteeringMeasurement)m;
				//TODO add locks
				currentAngle = (steerMeasur.getAngle())*Math.PI/180;
			}
		});
	}

	
	@Override
	protected void update() {
			
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
