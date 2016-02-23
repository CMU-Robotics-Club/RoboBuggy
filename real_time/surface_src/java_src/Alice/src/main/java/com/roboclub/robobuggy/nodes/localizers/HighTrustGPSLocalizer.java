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

/**
 *
 * This class runs a Node that will build a fused position estimate by trusting all new GPS measurements completely
 * @author Trevor Decker
 *
 */
public class HighTrustGPSLocalizer implements Node{
	private double buggyFrameGpsX;
	private double buggyFrameGpsY;
	private double buggyFrameRotZ;


	private Publisher posePub;

	/**
	 * Constructor for the High Trust Localizer which will initialize the system to an identity (zero position) 
	 */
	public HighTrustGPSLocalizer(){
		//init values
		buggyFrameGpsX = 0.0;
		buggyFrameGpsY = 0.0;


		//Initialize subscriber to GPS measurements
		new Subscriber(NodeChannel.GPS.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				GpsMeasurement newGPSData = (GpsMeasurement)m;

				// Get the delta latitude and longitude, use that to figure out how far we've travelled
				double oldGPSX = buggyFrameGpsX;
				double oldGPSY = buggyFrameGpsY;
				buggyFrameGpsY = newGPSData.getLongitude();
				buggyFrameGpsX = newGPSData.getLatitude();
				double dy = buggyFrameGpsY - oldGPSY;
				double dx = buggyFrameGpsX - oldGPSX;

				// take the arctangent in order to get the heading (in degrees)
				buggyFrameRotZ = Math.toDegrees(Math.atan2(dy,dx));

				publishUpdate();
			}
		});

	}

	private void publishUpdate(){
		posePub.publish(new GPSPoseMessage(new Date(), buggyFrameGpsX, buggyFrameGpsY, buggyFrameRotZ));
	}

	@Override
	public boolean startNode() {
		// TODO Auto-generated method stub
		posePub = new Publisher(NodeChannel.POSE.getMsgPath());;
		return true;
	}

	@Override
	public boolean shutdown() {
		posePub = null;
		buggyFrameGpsX = 0.0;
		buggyFrameGpsY = 0.0;
		return true;
	}

	@Override
	public void setName(String newName) {

	}

	@Override
	public String getName() {
		return "High Trust GPS Localizer";
	}



}
