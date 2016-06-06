package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
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
    private double lastEncoderReading;
    private double buggySteeringAngle;
    private double oldGPSX = 0.0;
    private double oldGPSY = 0.0;


    private Publisher posePub;

    /**
     * Constructor for the High Trust Localizer which will initialize the system to an identity (zero position)
     */
    public HighTrustGPSLocalizer(){
        //init values
    	buggyFrameGpsX = 0.0;
    	buggyFrameGpsY = 0.0;
        buggyFrameRotZ = 0.0;
        buggySteeringAngle = 0.0;// wheel direction in buggy frame
        lastEncoderReading = 0.0;
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());

        //Read all the steering messages we're getting and adjust our stored steering angle accordingly
        new Subscriber("htGpsLoc", NodeChannel.STEERING.getMsgPath(), new MessageListener() {			
			@Override
			public void actionPerformed(String topicName, Message m) {
				SteeringMeasurement steerM = (SteeringMeasurement)m;
				// TODO Auto-generated method stub
	            buggySteeringAngle  = steerM.getAngle();
			}
		});
        
        //Initialize subscriber to GPS measurements
        new Subscriber("htGpsLoc", NodeChannel.GPS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                GpsMeasurement newGPSData = (GpsMeasurement)m;
                synchronized (this) {
                // Get the delta latitude and longitude, use that to figure out how far we've travelled
               buggyFrameGpsY = newGPSData.getLatitude();
               buggyFrameGpsX = newGPSData.getLongitude();
               double dLat = buggyFrameGpsY - oldGPSY;
               double dLon = buggyFrameGpsX - oldGPSX;
               oldGPSX = buggyFrameGpsX;
               oldGPSY = buggyFrameGpsY;

                // take the arctangent in order to get the heading (in degrees)
                buggyFrameRotZ = Math.toDegrees(Math.atan2(LocalizerUtil.convertLatToMeters(dLat), LocalizerUtil.convertLonToMeters(dLon)));

                        publishUpdate();
                    }
                }
        });

 /*
        new Subscriber("HighTrustGpsLoc",NodeChannel.IMU_ANG_POS.getMsgPath(), ((topicName, m) -> {
            IMUAngularPositionMessage mes = ((IMUAngularPositionMessage) m);
//            double y = mes.getRot()[0][1];
//            double x = mes.getRot()[0][0];
//
//          buggyFrameRotZ = Util.normalizeAngleDeg(-Math.toDegrees(Math.atan2(y, x))+90);

            Matrix r = new Matrix(mes.getRot());
            double[][] xVec = {{1}, {0}, {0}};
            double[][] yVec = {{0}, {1}, {0}};

            double x = r.times(new Matrix(xVec)).get(0, 0);
           double y = r.times(new Matrix(yVec)).get(0, 0);
            buggyFrameRotZ = Util.normalizeAngleDeg(Math.toDegrees(-Math.atan2(y, x))+90 );


           publishUpdate();
        }));
        */
        

        //Update our position based on encoder readings
        
        // TODO note that we will probably run into precision errors since the changes are so small
        // would be good to batch up the encoder updates until we get a margin that we know can be represented proeprly
        new Subscriber("htGpsLoc", NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {

                EncoderMeasurement measurement = (EncoderMeasurement) m;

                // convert the feet from the last message into a delta degree, and update our position
                double currentEncoderMeasurement = measurement.getDistance();
                double deltaDistance = currentEncoderMeasurement - lastEncoderReading;
                // update heading around curve
                buggyFrameRotZ += MotionModel.getHeadingChange(deltaDistance, buggySteeringAngle);
                // advance by foward in new heading
                LocTuple deltaPos = LocalizerUtil.convertMetersToLatLng(deltaDistance, buggyFrameRotZ); //This line throws errors because LocalizerUtil doesn't have a LocTuple defined
                //But LocTuple (which supposedly had been in com.roboclub.robobuggy.ui) doesn't seem to exist anymore. Not sure what's supposed to be happening here.
                
                //Update our position estimate
                buggyFrameGpsY += deltaPos.getLatitude();
                buggyFrameGpsX += deltaPos.getLongitude();
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
