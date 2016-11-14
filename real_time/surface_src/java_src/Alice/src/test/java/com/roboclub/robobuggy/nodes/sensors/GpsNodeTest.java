package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.nodes.localizers.LocTuple;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import junit.framework.TestCase;
import org.junit.After;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.lang.InterruptedException;

import java.nio.ByteBuffer;
import java.nio.charset.Charset;
import java.util.Date;
import java.util.concurrent.LinkedBlockingQueue;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

/**
 * Created by abhinavgirish on 9/30/16.
 */
public class GpsNodeTest {
    private static LinkedBlockingQueue<GpsMeasurement> messageList = new LinkedBlockingQueue<>();

    @BeforeClass
    public static void oneTime() {
        new Subscriber("gpsLoc", NodeChannel.GPS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                messageList.add((GpsMeasurement) m);

            }

        });
    }

    @Before
    public void setUp() throws Exception {
        messageList.clear();
    }

    @After
    public void tearDown() throws Exception {

    }

    public void testName() throws Exception {

    }

    @Test
    public void testStandardGPSNode()
    {
        // to do - move subscriber stuff to oneTimeSetup(), call peel with the sample input, let it sleep for a
        // seconds, then check the LinkedBlockingQueue for messages
      GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS, "");
        //testing peel
        String input = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        byte[] bytes = Charset.forName("UTF-8").encode(input).array();
        gpsNode1.peel(bytes,0,bytes.length);
        try {
            Thread.sleep(3000);
            if (messageList.size() != 1) {
                fail("Did not receive message");
            }

            while (!messageList.isEmpty()) {
                GpsMeasurement m = messageList.take();
                assertEquals(m.getLatitude(), 48.1173, 0.0);
                assertEquals(m.getLongitude(), 11.51667, 0.0001);
                assertEquals(m.getNorth(), true);
                assertEquals(m.getWest(), false);
            }

            //check stuff in LinkedBlockingQueue
        }
        catch (Exception e){
                fail("Exception");
        }


    }

    /**
     * tests for correct failure if given longitude with invalid character
     */
    @Test
    public void testBadLong()
    {
        GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        String input = "$GPGGA,123519,4807.038,N,011A1.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        byte[] bytes = Charset.forName("UTF-8").encode(input).array();


        try {
            gpsNode1.peel(bytes,0,bytes.length);

            fail("This shouldn't parse longitude");

            //check stuff in LinkedBlockingQueue
        }
        catch (Exception e){
            return;
        }
    }

    /**
     * tests for correct failure if given latitude with invalid character
     */
    @Test
    public void testBadLat()
    {
        GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        String input = "$GPGGA,123519,48I7.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        byte[] bytes = Charset.forName("UTF-8").encode(input).array();


        try {
            gpsNode1.peel(bytes,0,bytes.length);

            fail("This shouldn't parse latitude");
            //check stuff in LinkedBlockingQueue
        }
        catch (Exception e){
            return;
        }
    }

    /**
     * tests for correct failure if given latitude and longitude with invalid characters
     */
    @Test
    public void testBadLatAndLong()
    {
        GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        String input = "$GPGGA,123519,48I7.038,N,011R1.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        byte[] bytes = Charset.forName("UTF-8").encode(input).array();


        try {
            gpsNode1.peel(bytes,0,bytes.length);

            fail("This shouldn't parse latitude and longitude");
            //check stuff in LinkedBlockingQueue
        }
        catch (Exception e){
            return;
        }
    }





}