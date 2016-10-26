package com.roboclub.robobuggy.messages;

import com.google.protobuf.Timestamp;
import com.roboclub.robobuggy.messages.Message.DataMessage;
import com.roboclub.robobuggy.messages.Message.ImuMessage;
import junit.framework.TestCase;
import org.junit.Before;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;


/**
 * Created by vivaanbahl on 10/25/16.
 */
public class MessageTests extends TestCase {

    private long testTimeSeconds;
    private int testTimeNanos;

    @Before
    protected void setUp() {
        long testTime = System.currentTimeMillis();
        testTimeSeconds = testTime / 1000;
        testTimeNanos = (int) ((testTime % 1000) * 1000000);
    }

    /**
     * Tests whether a Message can retain it's fields
     */
    @Test
    public void testRetentionTestIMU() {

        // test the IMU messages
        double testRoll = 3.14159;
        double testPitch = 1.242345;
        double testYaw = 2.7181;

        ImuMessage imuMessage = ImuMessage.newBuilder()
                .setRoll(testRoll)
                .setPitch(testPitch)
                .setYaw(testYaw)
                .setTime(Timestamp.newBuilder().setSeconds(testTimeSeconds).setNanos(testTimeNanos))
                .build();

        DataMessage.Builder testMessageBuilder = DataMessage.newBuilder();
        testMessageBuilder.setImu(imuMessage);
        DataMessage testMessage = testMessageBuilder.build();

        assertEquals(testMessage.getImu().getRoll(), testRoll);
        assertEquals(testMessage.getImu().getPitch(), testPitch);
        assertEquals(testMessage.getImu().getYaw(), testYaw);
        assertEquals(testMessage.getImu().getTime().getSeconds(), testTimeSeconds);
        assertEquals(testMessage.getImu().getTime().getNanos(), testTimeNanos);

    }

    @Test
    public void testSerialization() {

        DataMessage message = DataMessage.newBuilder()
                .setStatus(
                        Message.LogMessage.newBuilder()
                                .setLogLevel(Message.LogMessage.LogLevel.DEBUG)
                                .setText("Hello World!")
                                .setTime(Timestamp.newBuilder().setSeconds(testTimeSeconds).setNanos(testTimeNanos))
                )
                .build();

        ByteArrayOutputStream stream = new ByteArrayOutputStream();

        try {
            message.writeTo(stream);
            stream.close();

            DataMessage serialized = DataMessage.parseFrom(stream.toByteArray());

            assertEquals(serialized.getStatus().getText(), message.getStatus().getText());
            assertEquals(serialized.getStatus().getTime(), message.getStatus().getTime());
            assertEquals(serialized.getStatus().getLogLevel(), message.getStatus().getLogLevel());

        } catch (IOException e) {
            e.printStackTrace();
            fail("Couldn't parse serialized protobuf");
        }

    }

}