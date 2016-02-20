package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

import java.util.Date;

/**
 * Treats each gps value we get as true - good way for quick prototyping
 */
public class HighTrustLocalizer {
	private Publisher posePub;
	private double wheelOrientationBuggyFrame;
	private double buggyFrameGpsX;
	private double buggyFrameGpsY;
	private double buggyFrameGpsZ;
	
	HighTrustLocalizer(){
		//init values
		wheelOrientationBuggyFrame = 0.0;
		buggyFrameGpsX = 0.0;
		buggyFrameGpsY = 0.0;
		buggyFrameGpsZ = 0.0;
		wheelOrientationBuggyFrame = 0.0;
		
		//add subscribers 
		//Initialize subscriber to encoder measurements
		new Subscriber(NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
				@Override
				public void actionPerformed(String topicName, Message m) {
								
					}
				});

		posePub = new Publisher(NodeChannel.POSE.getMsgPath());
	}


	/**
	 * Sends a new pose message on the publishers
	 * @param lat pose's latitude
	 * @param lon pose's longitude
	 * @param heading pose's heading
	 */
	public void sendNewPoseMessage(double lat, double lon, double heading) {
		posePub.publish(new GPSPoseMessage(new Date(), lat, lon, heading));
	}

	/**
	 * @return the wheel orientation
	 */
	public double getWheelOrientationBuggyFrame() {
		return wheelOrientationBuggyFrame;
	}

	/**
	 * @param wheelOrientationBuggyFrame new wheel orientation
	 */
	public void setWheelOrientationBuggyFrame(double wheelOrientationBuggyFrame) {
		this.wheelOrientationBuggyFrame = wheelOrientationBuggyFrame;
	}

	/**
	 * @return the frame gps x coord
	 */
	public double getBuggyFrameGpsX() {
		return buggyFrameGpsX;
	}

	/**
	 * @param buggyFrameGpsX new frame gps x coord
	 */
	public void setBuggyFrameGpsX(double buggyFrameGpsX) {
		this.buggyFrameGpsX = buggyFrameGpsX;
	}

	/**
	 * @return the frame gps y coord
	 */
	public double getBuggyFrameGpsY() {
		return buggyFrameGpsY;
	}

	/**
	 * @param buggyFrameGpsY new frame gps y coord
	 */
	public void setBuggyFrameGpsY(double buggyFrameGpsY) {
		this.buggyFrameGpsY = buggyFrameGpsY;
	}

	/**
	 * @return the frame gps z coord
	 */
	public double getBuggyFrameGpsZ() {
		return buggyFrameGpsZ;
	}

	/**
	 * @param buggyFrameGpsZ new frame gps z coord
	 */
	public void setBuggyFrameGpsZ(double buggyFrameGpsZ) {
		this.buggyFrameGpsZ = buggyFrameGpsZ;
	}
}
