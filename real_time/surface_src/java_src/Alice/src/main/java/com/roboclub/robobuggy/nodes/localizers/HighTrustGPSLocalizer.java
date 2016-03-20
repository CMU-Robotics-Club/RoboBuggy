package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

import java.util.Date;

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
    private double roll = 0;
    private double pitch = 0;


    private Publisher posePub;

    /**
     * Constructor for the High Trust Localizer which will initialize the system to an identity (zero position)
     */
    public HighTrustGPSLocalizer(){
        //init values
        buggyFrameGpsX = 0.0;
        buggyFrameGpsY = 0.0;
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());



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
        
        new Subscriber(NodeChannel.IMU.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				ImuMeasurement imuM = (ImuMeasurement)m;
				//updates roll,pitch,yaw
				roll = Math.PI*imuM.getRoll()/180;
				pitch = Math.PI*imuM.getPitch()/180;
			}
		});
        
        new Subscriber(NodeChannel.IMU_MAGNETIC.getMsgPath(),new MessageListener() {	 			
        	 			@Override
        	 			public void actionPerformed(String topicName, Message m) {
        	 				MagneticMeasurement magM = (MagneticMeasurement)m;
        	 			
        	 				// Tilt compensated magnetic field X
        	 				  double magX = magM.getX() * Math.cos(pitch) + magM.getY() * Math.sin(roll) * Math.sin(pitch)
        	 						  + magM.getZ() * Math.cos(roll) * Math.sin(pitch);
        	 				  // Tilt compensated magnetic field Y
        	 				  double magY = magM.getY() * Math.cos(roll) - magM.getZ() * Math.sin(roll);
        	 				
        	 				double currAngle = -180*Math.atan2(-magY, magX)/Math.PI;
        	 				double offset = 0.0;
        	 				buggyFrameRotZ = currAngle - offset;
        	 				publishUpdate();
        	 				//TODO add a calibration step 
        	 			}
        	 		});

    }

    private void publishUpdate(){
        posePub.publish(new GPSPoseMessage(new Date(), buggyFrameGpsX, buggyFrameGpsY, buggyFrameRotZ));
    }

    @Override
    public boolean startNode() {
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
