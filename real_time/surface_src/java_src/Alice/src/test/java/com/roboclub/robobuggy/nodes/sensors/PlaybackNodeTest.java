package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.messages.BatteryLevelMessage;
import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.concurrent.LinkedBlockingQueue;

import static org.junit.Assert.fail;

/**
 * Created by vivaanbahl on 7/27/16.
 */
public class PlaybackNodeTest {

    private ArrayList<Subscriber> subList;
    private PlaybackNode playbackNode;
    private ArrayList<Class> messagesExpected;
    private LinkedBlockingQueue<Message> messagesReceived;

    /**
     * Creates a log with the specified trace, throws failure if file couldn't be created
     * @param logName name of the log
     * @param trace the formatted String the log will contain
     */
    public void createLogWithTrace(String logName, String trace) throws IOException {
        String log =
                "{\"name\":\"Robobuggy Data Logs\",\"schema_version\":2.0,\"date_recorded\":\"" +
                        new SimpleDateFormat(LoggingNode.DATE_FILE_FORMAT).format(new Date()) + "\"," +
                        "\"software_version\":\"qqqqqq\"}\n" +
                trace +
                "{\"VERSION_ID\":\"STOP\"}\n" +
                "{\"VERSION_ID\":\"data_breakdown\",\"logging_button\":0,\"gps\":0,\"imu\":0,\"encoder\":" +
                        "0,\"brake_state\":0,\"steering\":0,\"fp_hash\":0,\"logic_notification\":0," +
                        "\"push_bar_camera\":0,\"pose\":0,\"reset\":0,\"state\":0}";


        File f = new File(logName);
        if (!f.createNewFile()) {
            fail("Couldn't create log file!");
        }
        PrintWriter w = new PrintWriter(f, "UTF-8");
        w.write(log);
        w.close();
    }

    /**
     * Deletes the log specified, throws failure if file could not be deleted
     * @param logName the name of the log to be deleted
     */
    public void deleteLogFile(String logName) {
        File f = new File(logName);
        if (!f.delete()) {
            fail("Cleanup failed!");
        }
    }
    
    @Before
    public void setUp() {
        subList = new ArrayList<>();
        messagesExpected = new ArrayList<>();
        messagesReceived = new LinkedBlockingQueue<>();

        for (NodeChannel c : NodeChannel.getLoggingChannels()) {
            subList.add(new Subscriber("PlaybackNodeTest", c.getMsgPath(), (topicName, m) -> {
                if (!messagesReceived.offer(m)) {
                    System.err.println("Couldnt add received message :(");
                }
            }));
        }
    }

    @After
    public void tearDown() {

    }

    /**
     * Checks to see if all the messages expected were received, and that all messages received were expected
     * Fails if not
     */
    public void assertCorrectMessagesPlayed() {
        for (Message m : messagesReceived) {
            if (m  instanceof RobobuggyLogicNotificationMeasurement) {
                continue;
            }

            Class c = messagesExpected.remove(0);
            if (!m.getClass().equals(c)) {
                fail("Expected type " + c.toString() + " got " + m.getClass());
            }
        }

        if (!messagesExpected.isEmpty()) {
            fail("Didn't get all messages expected");
        }
    }

    /**
     * Plays back an empty log file
     * By empty we mean that the header is there, the footer is there, the STOP is there, but there are no messages logged
     */
    @Test
    public void testPlaybackEmptyLog() throws IOException, InterruptedException {
        createLogWithTrace("empty.txt", "");
        playbackNode = new PlaybackNode("empty.txt");
        playbackNode.startPlayback();
        Thread.sleep(1000);
        deleteLogFile("empty.txt");
        assertCorrectMessagesPlayed();
    }

    /**
     * Plays back a log file with only one sensor
     */
    @Test
    public void testPlaybackOnePubLog() throws IOException, InterruptedException {
        String msgAddition = "";
        for (int i = 0; i < 10; i++) {
            msgAddition = msgAddition.concat("{\"VERSION_ID\":\"" + SteeringMeasurement.VERSION_ID + "\",\"angle\":" + i +
                    ".0,\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\",\"timestamp\":0," +
                    "\"topicName\":\"sensors/steering\"," +
                    "\"sequenceNumber\":1}\n");

            messagesExpected.add(SteeringMeasurement.class);
        }

        createLogWithTrace("onePub.txt", msgAddition);
        playbackNode = new PlaybackNode("onePub.txt");
        playbackNode.startPlayback();
        Thread.sleep(6000);
        deleteLogFile("onePub.txt");
        assertCorrectMessagesPlayed();
    }

    /**
     * Plays back a standard log file
     */
    @Test
    public void testPlaybackStandardLog() throws InterruptedException, IOException {
        String msgAddition = "";
        for (int i = 0; i < 10; i++) {
            msgAddition = msgAddition.concat("{\"VERSION_ID\":\"" + SteeringMeasurement.VERSION_ID + "\",\"angle\":" + i +
                    ".0,\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\",\"timestamp\":12345678," +
                    "\"topicName\":\"sensors/steering\",\"sequenceNumber\":1}\n");
            msgAddition = msgAddition.concat("{\"VERSION_ID\":\"" + BrakeMessage.VERSION_ID + "\",\"down\":false," +
                    "\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\",\"timestamp\":12345678,\"topicName\":\"Topic\"," +
                    "\"sequenceNumber\":1}\n");
            msgAddition = msgAddition.concat("{\"VERSION_ID\":\"" + BatteryLevelMessage.VERSION_ID + "\",\"level\":" + i +
                    "," +
                    "\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\",\"timestamp\":12345678,\"topicName\":\"Topic\"," +
                    "\"sequenceNumber\":1}\n");

            messagesExpected.add(SteeringMeasurement.class);
            messagesExpected.add(BrakeMessage.class);
            messagesExpected.add(BatteryLevelMessage.class);
        }

        createLogWithTrace("standard.txt", msgAddition);
        playbackNode = new PlaybackNode("standard.txt");
        playbackNode.startPlayback();
        Thread.sleep(4000);
        deleteLogFile("standard.txt");
        assertCorrectMessagesPlayed();
    }

}