package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.concurrent.LinkedBlockingQueue;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.fail;

/**
 * Created by vivaanbahl on 2/15/17.
 */
public class RBSMNodeTest {
    private static LinkedBlockingQueue<Message> messages;
    private RBSMNode testNode;

    private static final byte RBSM_MESSAGE_FOOTER = 0xA;

    /**
     * Runs setup for the test class
     */
    @BeforeClass
    public static void oneTimeSetup() {
        messages = new LinkedBlockingQueue<>();
        new Subscriber("RBSM Node Unit Tests", NodeChannel.LOGIC_NOTIFICATION.getMsgPath(), (topicName, m) -> {
            messages.add(m);
        });
    }

    /**
     * Runs setup for each test case
     */
    @Before
    public void setUp() {
        // get a dummy RBSMNode each time
        testNode = new RBSMNode(NodeChannel.STEERING, NodeChannel.STEERING, null, 100);
    }

    /**
     * Tests the RBSM MID error translation from the bit representation to the
     * error message
     *
     * Iterates through each known low-level error and calls peel() with a faked message
     *
     * Compares the output of peel() with the expected RBSMErrorCodes message format
     *
     * No errors expected
     */
    @Test
    public void testRbsmMidErrorTranslation() {
        byte rbsmMidErrorHeader = (byte) 254;
        byte[] fakeBuffer = { rbsmMidErrorHeader, 0, 0, 0, 0, RBSM_MESSAGE_FOOTER };

        // test all ok, expect no response
        testNode.peel(fakeBuffer, 0, 6);
        // wait for the peel method to propagate
        waitForMessagePropagation();
        // system ok shouldn't generate anything
        assertEquals(messages.size(), 0);

        // test other error ids
        // loop over the other values
        // NOTE we skip the first one since it's a special case
        for (int i = 1; i < RBSMNode.RBSMErrorCodes.values().length; i++) {
            // get the ith error code, and put it in the buffer
            RBSMNode.RBSMErrorCodes errorCode = RBSMNode.RBSMErrorCodes.values()[i];
            fakeBuffer[4] = (byte) errorCode.getErrorCode();
            testNode.peel(fakeBuffer, 0, 6);

            waitForMessagePropagation();

            // make sure that we received a message
            Message top = messages.poll();
            assertNotNull(top);
            assertEquals(((RobobuggyLogicNotificationMeasurement) top).getMessage(),
                    "RBSM_MID_ERROR:" + errorCode.getErrorCode() + ": " + errorCode.getErrorMessage());
        }

    }

    /**
     * Unfortunately we can't trigger continuation very easily, so we instead just wait
     * for 100 ms for peel() to finish
     */
    private void waitForMessagePropagation() {
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
            fail("Thread was interrupted");
        }
    }

}