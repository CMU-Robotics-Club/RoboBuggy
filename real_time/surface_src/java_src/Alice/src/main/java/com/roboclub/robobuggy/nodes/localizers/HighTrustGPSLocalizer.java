package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.LocTuple;

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
    private Date mostRecentUpdate;
    //private double roll = 0;
    //private double pitch = 0;

    private static final double FEET_TO_METERS = 0.3048;

    private Publisher posePub;
    private double lastEncoderMeasurement;

    /**
     * Constructor for the High Trust Localizer which will initialize the system to an identity (zero position)
     */
    public HighTrustGPSLocalizer(){
        //init values
        buggyFrameGpsX = 0.0;
        buggyFrameGpsY = 0.0;
        buggyFrameRotZ = 0.0;
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());
        mostRecentUpdate = new Date();


        //Initialize subscriber to GPS measurements
        new Subscriber(NodeChannel.GPS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
            	GpsMeasurement newGPSData = (GpsMeasurement)m;
            	synchronized (this) {
            	long dt = newGPSData.getTimestamp().getTime() - mostRecentUpdate.getTime();
            	System.out.println(dt);
            	if(dt > 0.0){
                // Get the delta latitude and longitude, use that to figure out how far we've travelled

                double oldGPSX = buggyFrameGpsX;
                double oldGPSY = buggyFrameGpsY;
                	buggyFrameGpsY = newGPSData.getLatitude();
                	buggyFrameGpsX = newGPSData.getLongitude();
                double dLat = buggyFrameGpsY - oldGPSY;
                double dLon = buggyFrameGpsX - oldGPSX;
                
                double oldRotZ = buggyFrameRotZ;

                // take the arctangent in order to get the heading (in degrees)
                buggyFrameRotZ = Math.toDegrees(Math.atan2(dLat, -dLon));


                publishUpdate();
            	mostRecentUpdate = newGPSData.getTimestamp();
            	}
                }
            }
        });
        
//        new Subscriber(NodeChannel.IMU.getMsgPath(), new MessageListener() {
//
//			@Override
//			public void actionPerformed(String topicName, Message m) {
//				ImuMeasurement imuM = (ImuMeasurement)m;
//				//updates roll,pitch,yaw
//				roll = Math.PI*imuM.getRoll()/180;
//				pitch = Math.PI*imuM.getPitch()/180;
//				yaw = Math.PI*imuM.getYaw()/180;
//			}
//		});
//
//        new Subscriber(NodeChannel.IMU_MAGNETIC.getMsgPath(),new MessageListener() {
//        	 			@Override
//        	 			public void actionPerformed(String topicName, Message m) {
//        	 				MagneticMeasurement magM = (MagneticMeasurement)m;
//
//        	 				// Tilt compensated magnetic field X
//        	 				  double mag_x = magM.getX() * Math.cos(pitch) + magM.getY() * Math.sin(roll)
// * Math.sin(pitch) + magM.getZ() * Math.cos(roll) * Math.sin(pitch);
//        	 				  // Tilt compensated magnetic field Y
//        	 				  double mag_y = magM.getY() * Math.cos(roll) - magM.getZ() * Math.sin(roll);
//
//        	 				double currAngle = -180*Math.atan2(-mag_y, mag_x)/Math.PI;
//        	 				//TODO stop this from being a 5:29 am hack
//        	 				double offset = 0.0;
//        	 				buggyFrameRotZ = currAngle - offset;
//        	 				publishUpdate();
//        	 				//TODO add a calibration step
//        	 			}
//        	 		});

        // TODO note that we will probably run into precision errors since the changes are so small
        // would be good to batch up the encoder updates until we get a margin that we know can be represented proeprly
        new Subscriber(NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                EncoderMeasurement measurement = (EncoderMeasurement) m;

                // convert the feet from the last message into a delta degree, and update our position
                double currentEncoderMeasurement = measurement.getDistance();
                double deltaDistance = currentEncoderMeasurement - lastEncoderMeasurement;
                double deltaMeters = deltaDistance * FEET_TO_METERS;

                LocTuple deltaPos = LocalizerUtil.convertMetersToLatLng(deltaMeters, buggyFrameRotZ);
                buggyFrameGpsY += deltaPos.getLatitude();
                buggyFrameGpsX += deltaPos.getLongitude();

                lastEncoderMeasurement = currentEncoderMeasurement;

                publishUpdate();
            }
        });

    }

    private void publishUpdate(){
        posePub.publish(new GPSPoseMessage(new Date(), buggyFrameGpsY, buggyFrameGpsX, buggyFrameRotZ));
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
