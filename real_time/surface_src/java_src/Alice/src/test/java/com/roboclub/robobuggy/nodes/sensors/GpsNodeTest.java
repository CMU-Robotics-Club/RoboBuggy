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

import java.util.Date;
import java.util.concurrent.LinkedBlockingQueue;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

/**
 * Created by abhinavgirish on 9/30/16.
 */
public class GpsNodeTest {
    private static LinkedBlockingQueue<GpsMeasurement> messageList = new LinkedBlockingQueue<>();
    private LocTuple lastReading;
    private Publisher pub; //figure out relationship between publisher, subscriber etc.

    @BeforeClass
    public static void oneTime() {
        new Subscriber("gpsLoc", NodeChannel.GPS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                GpsMeasurement gpsM = (GpsMeasurement) m;
                messageList.add((GpsMeasurement) m);

                //Feed the watchdog
                //setNodeState(NodeState.ON);
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
      GpsNode gpsNode1 = new GpsNode(NodeChannel.GPS,"buggy");
        //testing peel
        String input = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
        char[] inputChars = input.toCharArray();
        byte[] bytes = new String(inputChars).getBytes();
        gpsNode1.peel(bytes,0,bytes.length);
        try {
            Thread.sleep(3000);
            if (messageList.size() != 1) {
                fail("Did not receive message");
            }
            Thread.sleep(3000);

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

        }


    }

}