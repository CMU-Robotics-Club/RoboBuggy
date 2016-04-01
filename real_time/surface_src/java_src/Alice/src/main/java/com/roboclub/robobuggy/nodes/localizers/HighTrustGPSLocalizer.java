package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;

import java.sql.Timestamp;
import java.util.Date;

/**
 *
 * This class runs a Node that will build a fused position estimate by trusting all new GPS measurements completely
 * @author Trevor Decker
 *
 */
public class HighTrustGPSLocalizer implements Node{
    private double buggyFrameGpsLon;
    private double buggyFrameGpsLat;
    private double buggyFrameRotZ;
    private Date mostRecentUpdate;
    private double lastEncoderReading;
    //private double roll = 0;
    //private double pitch = 0;


    private Publisher posePub;

    /**
     * Constructor for the High Trust Localizer which will initialize the system to an identity (zero position)
     */
    public HighTrustGPSLocalizer(){
        //init values
        buggyFrameGpsLon = 0.0;
        buggyFrameGpsLat = 0.0;
        lastEncoderReading = 0.0;
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());
        mostRecentUpdate = new Date();


        //Initialize subscriber to GPS measurements
        new Subscriber(NodeChannel.GPS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
            	GpsMeasurement newGPSData = (GpsMeasurement)m;
            	synchronized (this) {
            	long dt = newGPSData.getTimestamp().getTime() - mostRecentUpdate.getTime();
            	if(dt > 0.0){
                // Get the delta latitude and longitude, use that to figure out how far we've travelled

 //               double oldGPSX = buggyFrameGpsLon;
 //               double oldGPSY = buggyFrameGpsLat;
              //  buggyFrameGpsLat = newGPSData.getLatitude();
               // buggyFrameGpsLon = newGPSData.getLongitude();	
 //               double dLat = buggyFrameGpsLat - oldGPSY;
 //               double dLon = buggyFrameGpsLon - oldGPSX;
                
 //               double oldRotZ = buggyFrameRotZ;

                publishUpdate();
            	mostRecentUpdate = newGPSData.getTimestamp();
            	}
                }
            }
        });
        
        new Subscriber(NodeChannel.IMU_MAGNETIC.getMsgPath(),new MessageListener() {
        	 			@Override
        	 			public void actionPerformed(String topicName, Message m) {
        	 				MagneticMeasurement magM = (MagneticMeasurement)m;
        	 				double currAngle = magM.getRotationZ();
        	 				double offset = 0.0;
        	 				buggyFrameRotZ = currAngle - offset;
        	 				publishUpdate();
        	 				//TODO add a calibration step
        	 			}
        	 		});
        
        new Subscriber(NodeChannel.ENCODER.getMsgPath(),new MessageListener() {
 			@Override
 			public void actionPerformed(String topicName, Message m) {
 				EncoderMeasurement magM = (EncoderMeasurement)m;
 				double dEncoder = magM.getDistance() - lastEncoderReading;
 				lastEncoderReading = magM.getDistance();
 				buggyFrameGpsLon = buggyFrameGpsLon + dEncoder*Math.cos(buggyFrameRotZ) + dEncoder*Math.sin(buggyFrameRotZ);
 				buggyFrameGpsLat = -buggyFrameGpsLat + dEncoder*Math.sin(buggyFrameRotZ) + dEncoder*Math.cos(buggyFrameRotZ);
 				publishUpdate();
 				//TODO add a calibration step
 			}
 		});    
        
    }

    private void publishUpdate(){
        posePub.publish(new GPSPoseMessage(new Date(), buggyFrameGpsLat, buggyFrameGpsLon, buggyFrameRotZ));
    }	

    @Override
    public boolean startNode() {
        return true;
    }

    @Override
    public boolean shutdown() {
        posePub = null;
        buggyFrameGpsLon = 0.0;
        buggyFrameGpsLat = 0.0;
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
