package com.roboclub.robobuggy.nodes.localizers;

import java.util.Date;

import com.roboclub.robobuggy.map.So2Pose;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
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
	private So2Pose pose;
	private Publisher posePub = new Publisher(NodeChannel.POSE.getMsgPath());

	private double mostRecentEncoder = 0;
	private double secondOldestEncoder = 0;
	private double mostRecentAngle = 0;
	private double secondOldestAngle = 0;
	
	
	public OdomLocalizer() {
		super(new  BuggyBaseNode(NodeChannel.POSE), 100);
		
		
		//Initialize subscriber to encoder measurements
		new Subscriber(NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
		@Override
		public void actionPerformed(String topicName, Message m) {
			EncoderMeasurement encM = (EncoderMeasurement)m;
			//TODO add locks 
			double dist = encM.getDistance();
			//to stop small value errors
			double delta = dist - mostRecentEncoder;
			//System.out.println("delta:"+delta);
			//if(Math.abs(delta) > .01){
				secondOldestEncoder = mostRecentEncoder;
				mostRecentEncoder = dist;
			if (pose == null) {
				pose = new So2Pose(0.0, 0.0, 0.0);
			}
			double deltaAngle = mostRecentAngle - secondOldestAngle;
			double deltaEncoder = mostRecentEncoder - secondOldestEncoder;
			So2Pose deltaPose = new So2Pose(deltaEncoder, 0.0, deltaAngle);
			pose = pose.mult(deltaPose);
			posePub.publish(new GPSPoseMessage(new Date(), pose.getX(), pose.getY(), pose.getOrintation()));
			//posePub.publish(new PoseMessage(new Date(), mostRecentEncoder, 0, 0));
			System.out.println("x:"+pose.getX()+"\t y:"+pose.getY()+"\t orintation"+pose.getOrintation());
			secondOldestAngle = mostRecentAngle;
			//}
			
						
			}
		});
			
		//Initialize subscriber for what steering angle low level is currently at
		new Subscriber(NodeChannel.STEERING.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				SteeringMeasurement steerMeasur =  (SteeringMeasurement)m;
				//TODO add locks
				mostRecentAngle = (steerMeasur.getAngle())*Math.PI/180;
				System.out.println("angle:"+mostRecentAngle);
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
