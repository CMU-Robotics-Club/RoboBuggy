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

import java.util.Date;

/**
 *
 * This class runs a Node that will build a fused position estimate by trusting all new GPS measurements completely
 * @author Trevor Decker
 *
 */
public class HighTrustLocalizer implements Node {
    private double buggyFrameGpsLat;
    private double buggyFrameGpsLon;
    private double buggyFrameHeading;

    private Publisher posePub;

    private double lastEncoderMeasurement;

    // some math for the conversion of latitude to meters
    private static final double ONE_DEG_LAT_TO_METERS_EQUATOR = 111319.9;
    private static final double ONE_DEG_LON_TO_METERS = ONE_DEG_LAT_TO_METERS_EQUATOR; // Lon doesn't depend on degree
    private static final double LAT_DEGREE_PITTSBURGH = 40.440310; // center of Schenley
    private static final double ONE_DEG_LAT_TO_METERS_PITTSBURGH =
            ONE_DEG_LAT_TO_METERS_EQUATOR * Math.cos(Math.toRadians(LAT_DEGREE_PITTSBURGH));
    private static final double FEET_TO_METERS = 0.3048;

    /**
     * Constructor for the High Trust Localizer which will initialize the system to an identity (zero position)
     */
    public HighTrustLocalizer(){
        //init values
        buggyFrameGpsLat = 0.0;
        buggyFrameGpsLon = 0.0;
        lastEncoderMeasurement = 0;


        //Initialize subscriber to GPS measurements
        new Subscriber(NodeChannel.GPS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                GpsMeasurement newGPSData = (GpsMeasurement)m;
                buggyFrameGpsLat = newGPSData.getLatitude();
                buggyFrameGpsLon = newGPSData.getLongitude();
                // no verification yet - we are running high-trust

                //we're going to keep the same orientation as before

                publishUpdate();
            }
        });
        
        new Subscriber(NodeChannel.IMU_MAGNETIC.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				MagneticMeasurement magM = (MagneticMeasurement)m;
                // no verification yet - running in high trust mode
                buggyFrameHeading = Math.atan2(magM.getY(), magM.getX());

                // we're going to keep the same position as before

                publishUpdate();
			}
		});

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
                double deltaMetersX = deltaMeters * Math.cos(buggyFrameHeading);
                double deltaMetersY = deltaMeters * Math.sin(buggyFrameHeading);

                double deltaDegreesLat = deltaMetersY/ONE_DEG_LAT_TO_METERS_PITTSBURGH;
                double deltaDegreesLon = deltaMetersX/ONE_DEG_LON_TO_METERS;

                buggyFrameGpsLat += deltaDegreesLat;
                buggyFrameGpsLon += deltaDegreesLon;

                publishUpdate();
            }
        });

    }

    private void publishUpdate(){
        posePub.publish(new GPSPoseMessage(new Date(), buggyFrameGpsLat, buggyFrameGpsLon, buggyFrameHeading));
    }

    @Override
    public boolean startNode() {
        posePub = new Publisher(NodeChannel.POSE.getMsgPath());
        return true;
    }

    @Override
    public boolean shutdown() {
        posePub = null;
        buggyFrameGpsLat = 0.0;
        buggyFrameGpsLon = 0.0;
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
