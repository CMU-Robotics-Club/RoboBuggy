package com.roboclub.robobuggy.nodes.localizers;

import java.awt.Point;
import java.util.Date;

import com.roboclub.robobuggy.map.So2Pose;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

public class HighTrustLocalizer implements Node{
	private double wheelOrintation_buggyFrame;
	private double buggyFrame_gps_x;
	private double buggyFrame_gps_y;
	private double buggyFrame_rot_z;
	private double mostRecentEncoder = 0;
	private double secondOldestEncoder = 0;

	private double LATITUDE_ZERO = 40.44288816666667;
	private double LONGITUDE_ZERO = -79.9427065; 
	
	private Publisher posePub = new Publisher(NodeChannel.POSE.getMsgPath());

	
	public HighTrustLocalizer(){
		//init values
		wheelOrintation_buggyFrame = 0.0;
		buggyFrame_gps_x = 0.0;
		buggyFrame_gps_y = 0.0;
		wheelOrintation_buggyFrame = 0.0;

		
		//steering
		new Subscriber(NodeChannel.STEERING.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				SteeringMeasurement steerM = (SteeringMeasurement)m;
				wheelOrintation_buggyFrame = steerM.getAngle();
				
			}
		});
		
		//add subscribers 
		//Initialize subscriber to encoder measurements
		new Subscriber(NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
						EncoderMeasurement encM = (EncoderMeasurement)m;
						secondOldestEncoder = mostRecentEncoder;
						mostRecentEncoder = encM.getDistance();
						
						//Get orientation in world frame
						double worldOrintation = buggyFrame_rot_z+wheelOrintation_buggyFrame;
						//TODO move us forward by that amount 
						double deltaEncoder = mostRecentEncoder - secondOldestEncoder;
						So2Pose deltaPose = new So2Pose(deltaEncoder, 0.0, worldOrintation);
						com.roboclub.robobuggy.map.Point deltaPoint = deltaPose.getSe2Point();
					//	buggyFrame_gps_x = buggyFrame_gps_x + deltaPoint.getX();
					//	buggyFrame_gps_y= buggyFrame_gps_y + deltaPoint.getY();
						
						//now publisher the point
					//	publishUpdate();	

					}
				}); 
		
		//add subscribers 
		//Initialize subscriber to GPS measurements
		new Subscriber(NodeChannel.GPS.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
					  GpsMeasurement gpsM = (GpsMeasurement)m;
					  double thisLon = -gpsM.getLongitude();
					  double thisLat = gpsM.getLatitude();
					  double dLongitude = thisLon - LONGITUDE_ZERO;
					  double dLatitude= thisLat - LATITUDE_ZERO;

					  double oldX = buggyFrame_gps_x;
					  double oldY = buggyFrame_gps_y;
					  buggyFrame_gps_x = Math.cos(thisLon) * 69.172*5280*dLongitude;  
					  buggyFrame_gps_y = 365228*dLatitude;
					  double dx = buggyFrame_gps_x - oldX;
					  double dy = buggyFrame_gps_y - oldY;
					  buggyFrame_rot_z = 180*Math.atan2(dy, dx)/Math.PI;
						publishUpdate();	
					}
				});
		
		//add subscribers 
		//Initialize subscriber to IMU measurements
		new Subscriber(NodeChannel.IMU.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
						ImuMeasurement imuM = (ImuMeasurement)m;
						//TODO useful things
				//		publishUpdate();	

					}
				});
		
		//add subscribers 
		//Initialize subscriber to IMU MAGNITOMETER measurements
		new Subscriber(NodeChannel.IMU_MAGNETIC.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
								MagneticMeasurement magM = (MagneticMeasurement)m;
								//TODO useful things
				//				publishUpdate();	

					}
				});
		
	}

	private void publishUpdate(){
		posePub.publish(new GPSPoseMessage(new Date(), buggyFrame_gps_x, buggyFrame_gps_y, buggyFrame_rot_z));
	}

	@Override
	public boolean startNode() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean shutdown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void setName(String newName) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public String getName() {
		return "highTrustLocalizer";
	}
	
	
	
}
