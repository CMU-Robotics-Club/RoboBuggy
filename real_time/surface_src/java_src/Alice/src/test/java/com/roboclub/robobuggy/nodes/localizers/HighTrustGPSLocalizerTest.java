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

    private static final double latPittsburgh = 40.440310;
    private static final double lonPittsburgh = -79.9471537;
    private static final double calcedDistOneDegMeters = 84721.0;

    private Publisher gpsPub;
    private Publisher encoderPub;

    @Before
    public void setUp() throws Exception {
        gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
        encoderPub = new Publisher(NodeChannel.ENCODER.getMsgPath());
    }

    @After
    public void tearDown() throws Exception {
        gpsPub = null;
        encoderPub = null;
    }

    @Test
    public void test_distanceBetweenOneDegLat() throws InterruptedException {
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

        gpsPub.publish(new GpsMeasurement(latPittsburgh, lonPittsburgh));
        Thread.sleep(500);
        encoderPub.publish(new EncoderMeasurement(calcedDistOneDegMeters, 0.0));

    }

    public boolean isWithinTolerance(double val1, double val2, double tolerance) {
        return Math.abs(val1 - val2) < tolerance;

    }

}