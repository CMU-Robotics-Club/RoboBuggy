package com.roboclub.robobuggy.nodes.localizers;

import java.util.Date;

import com.roboclub.robobuggy.map.So2Pose;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

public class HighTrustLocalizer {
	private Publisher posePub;
	private double wheelOrintation_buggyFrame;
	private double buggyFrame_gps_x;
	private double buggyFrame_gps_y;
	private double buggyFrame_gps_z;
	
	HighTrustLocalizer(){
		//init values
		wheelOrintation_buggyFrame = 0.0;
		buggyFrame_gps_x = 0.0;
		buggyFrame_gps_y = 0.0;
		buggyFrame_gps_z = 0.0;
		wheelOrintation_buggyFrame = 0.0;
		
		//add subscribers 
		//Initialize subscriber to encoder measurements
		new Subscriber(NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
								
					}
				}); 
	}
	
	
	
	
}
