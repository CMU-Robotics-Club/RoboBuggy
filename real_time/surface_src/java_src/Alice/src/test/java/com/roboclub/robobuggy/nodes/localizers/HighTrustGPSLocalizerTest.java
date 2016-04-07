package com.roboclub.robobuggy.nodes.localizers;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

/**
 * An integration test of sorts... :)
 */
public class HighTrustGPSLocalizerTest {

    private static final double LAT_PITTSBURGH = 40.440310;
    private static final double LON_PITTSBURGH = -79.9471537;
    private static final double CALCED_DIST_ONE_DEG_METERS = 84721.0;

    private Publisher gpsPub;
    private Publisher encoderPub;

    /**
     * sets up the publishers
     */
    @Before
    public void setUp() {
        gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
        encoderPub = new Publisher(NodeChannel.ENCODER.getMsgPath());
    }

    /**
     * invalidates the publishers
     */
    @After
    public void tearDown() {
        gpsPub = null;
        encoderPub = null;
    }

    /**
     * tests whether the pose estimator can translate meters into latlng stuffs
     * @throws InterruptedException if timers somehow failed
     */
    @Test
    public void testDistanceBetweenOneDegLat() throws InterruptedException {
        final double[] lat1 = { 0.0 };
        final double[] lon1 = {0.0};
        final double[] lat2 = {0.0};
        final double[] lon2 = {0.0};

        new Subscriber(NodeChannel.POSE.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                GPSPoseMessage pm = (GPSPoseMessage) m;

                if (lat1[0] == 0.0) {
                    lat1[0] = pm.getLatitude();
                    lon1[0] = pm.getLongitude();
                }
                else {
                    lat2[0] = pm.getLatitude();
                    lon2[0] = pm.getLongitude();

                    Assert.assertEquals(lat1[0], lat2[0], 0.000001);
                    Assert.assertEquals(lon1[0] - 1, lon2[0], 0.00002);
                }

            }
        });

        gpsPub.publish(new GpsMeasurement(LAT_PITTSBURGH, LON_PITTSBURGH));
        Thread.sleep(500);
        encoderPub.publish(new EncoderMeasurement(CALCED_DIST_ONE_DEG_METERS, 0.0));

    }

}