package com.roboclub.robobuggy.nodes.localizers;

import Jama.Matrix;

import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.IMUAngularPositionMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
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
    private double lastEncoderReading;
    private double buggyHeading;
    private double oldGPSX;
    private double oldGPSY;


    private Publisher posePub;

    /**
     * Constructor for the High Trust Localizer which will initialize the system to an identity (zero position)
     */
    public HighTrustGPSLocalizer(){
        //init values
    	buggyFrameGpsX = 0.0;
    	buggyFrameGpsY = 0.0;
        buggyFrameRotZ = 0.0;
        buggyHeading = 0.0;// wheel direction in buggy frame
        lastEncoderReading = 0.0;
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());
        mostRecentUpdate = new Date();

        new Subscriber("htGpsLoc", NodeChannel.STEERING.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				SteeringMeasurement steerM = (SteeringMeasurement)m;
				// TODO Auto-generated method stub
	             buggyHeading  = steerM.getAngle();
	              publishUpdate();
			}
		});
        
        //Initialize subscriber to GPS measurements
        new Subscriber("htGpsLoc", NodeChannel.GPS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                GpsMeasurement newGPSData = (GpsMeasurement)m;
                synchronized (this) {
                    long dt = newGPSData.getTimestamp().getTime() - mostRecentUpdate.getTime();
                    if(dt > 0.0){
                          // Get the delta latitude and longitude, use that to figure out how far we've travelled
               buggyFrameGpsY = newGPSData.getLatitude();
               buggyFrameGpsX = newGPSData.getLongitude();
               double dLat = buggyFrameGpsY - oldGPSY;
               double dLon = buggyFrameGpsX - oldGPSX;
               oldGPSX = buggyFrameGpsX;
               oldGPSY = buggyFrameGpsY;

                

                // take the arctangent in order to get the heading (in degrees)
                  buggyFrameRotZ = Math.toDegrees(Math.atan2(dLat, dLon));
                  // System.out.println("latlng"+dLat+","+dLon);

                        publishUpdate();
                        mostRecentUpdate = newGPSData.getTimestamp();
                    }
                }
            }
        });

     
        new Subscriber("HighTrustGpsLoc",NodeChannel.IMU_ANG_POS.getMsgPath(), ((topicName, m) -> {
            IMUAngularPositionMessage mes = ((IMUAngularPositionMessage) m);
            //double y = mes.getRot()[0][1];
            //double x = mes.getRot()[0][0];
            double[][] xVar = {{1},{0},{0}};
            double[][] yVar = {{0},{1},{0}};
            Matrix xMat = new Matrix(xVar);
            Matrix yMat = new Matrix(yVar);
            Matrix rot = new Matrix(mes.getRot());
            double x = rot.times(xMat).get(0, 0);
            double y = rot.times(yMat).get(0, 0);
            double th = -(Math.toDegrees(Math.atan2(y, x))-90);
            buggyFrameRotZ = Util.normalizeAngleDeg((Util.normalizeAngleDeg(th)+buggyFrameRotZ)/2);
            
           publishUpdate();
        }));
        

      
        // TODO note that we will probably run into precision errors since the changes are so small
        // would be good to batch up the encoder updates until we get a margin that we know can be represented proeprly
        new Subscriber("htGpsLoc", NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {

                EncoderMeasurement measurement = (EncoderMeasurement) m;

                // convert the feet from the last message into a delta degree, and update our position
                double currentEncoderMeasurement = measurement.getDistance();
                double deltaDistance = currentEncoderMeasurement - lastEncoderReading;
              //  System.out.println("buggyHeading"+buggyHeading);
                LocTuple deltaPos = LocalizerUtil.convertMetersToLatLng(deltaDistance, buggyFrameRotZ+buggyHeading);
                buggyFrameGpsY += deltaPos.getLatitude();
                buggyFrameGpsX += deltaPos.getLongitude();
                buggyFrameRotZ = buggyFrameRotZ+buggyHeading;
                lastEncoderReading = currentEncoderMeasurement;

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
