package com.roboclub.robobuggy.messages;

import junit.framework.TestCase;
import org.junit.Test;

/**
 * Created by vivaanbahl on 10/25/16.
 */
public class MessageTests extends TestCase {
    Message.DataMessage testMessage;

    /**
     * Tests whether a Message can retain it's fields
     */
    @Test
    public void testRetentionTest() {
        double testRoll = 3.14159;
        double testPitch = 1.242345;
        double testYaw = 2.7181;

        Message.ImuMessage imuMessage = Message.ImuMessage.newBuilder()
                .setRoll(testRoll)
                .setPitch(testPitch)
                .setYaw(testYaw)
                .build();

        testMessage = Message.DataMessage.newBuilder()
                .setImu(imuMessage)
                .build();

        assertEquals(testMessage.getImu().getRoll(), testRoll);
        assertEquals(testMessage.getImu().getPitch(), testPitch);
        assertEquals(testMessage.getImu().getYaw(), testYaw);
    }

}