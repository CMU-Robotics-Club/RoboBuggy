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

    /**
     * tests that message is properly parsed for a legitimate input
     */
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

    /**
     * tests if convertHHMMSStoTime() method works correctly on valid input
     */
    @Test
    public void testConversionDateTime(){

        GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        String input = "123519";
        Date d1 = gpsNode1.convertHHMMSStoTime(input);
        assertEquals(d1.getHours(),12);
        assertEquals(d1.getMinutes(),35);
        assertEquals(d1.getSeconds(),19);
        System.out.println(d1);
        //posDDMMmmmm could correspond to a reading of 41d 24.8963' N

    }

    /**
     * tests if convertHHMMSStoTime() method works correctly on valid input
     */
    @Test
    public void testBadConversionDateTime(){

        GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        String input = "123L19";
        try {
            Date d1 = gpsNode1.convertHHMMSStoTime(input);
            fail("could not convert the time");
        } catch (AssertionError e){
            return;
        }

        //posDDMMmmmm could correspond to a reading of 41d 24.8963' N

    }

    /**
     * tests if convertMinutesSecondsToFloat() works correctly on valid input
     */
    @Test
    public void testConversionMinSecToFloat()
    {
        GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        String input = "4807.038";
        double output = gpsNode1.convertMinutesSecondsToFloat(input);
        System.out.println(output);
        assertEquals(output,48.1173,0.001);
    }

    /**
     * tests for correct failure if convertMinutesSecondsToFloat() is given invalid input
     */
    @Test
    public void testBadConversionMinSecToFloat()
    {
        GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        String input = "480O.038";
        try {
            double output = gpsNode1.convertMinutesSecondsToFloat(input);
            fail("cannot convert given latitude");
        } catch (Exception e){
            return;
        }
    }



    /**
     * tests if convertMinSecToFloatLongitude() works correctly on valid input
     */
    @Test
    public void testConversionMinSecToFloatLong()
    {
        GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        String input = "01131.000";
        double output = gpsNode1.convertMinSecToFloatLongitude(input);
        System.out.println(output);
        assertEquals(output,11.5167,0.001);
    }

    /**
     * tests for correct failure of convertMinSecToFloatLongitude()
     * if given invalid input
     */
    @Test
    public void testBadConversionMinSecToFloatLong()
    {
        GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        String input = "01E31.000";
        try
        {
            double output = gpsNode1.convertMinSecToFloatLongitude(input);
            fail("couldn't convert longitude - invalid input");

        } catch (Exception E){
            return;
        }
    }








}