package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

import static org.junit.Assert.fail;

/**
 * Created by vivaanbahl on 7/27/16.
 */
public class PlaybackNodeTest {

    private ArrayList<Subscriber> subList;
    private PlaybackNode playbackNode;
    private ArrayList<Message> messagesExpected;
    private ArrayList<Message> messagesReceived;

    /**
     * Creates a log with the specified trace, throws failure if file couldn't be created
     * @param logName name of the log
     * @param trace the formatted String the log will contain
     */
    public void createLogWithTrace(String logName, String trace) throws IOException {
        String log =
                "\\{\"name\":\"Robobuggy Data Logs\",\"schema_version\":2.0\"date_recorded\":\"" +
                        new SimpleDateFormat(LoggingNode.DATE_FILE_FORMAT).format(new Date()) + "\"," +
                        "\"software_version\":\"qqqqqq\"\\}\n" +
                trace +
                "\\{\"VERSION_ID\":\"STOP\"\\}\n" +
                "\\{\"VERSION_ID\":\"data_breakdown\",\"logging_button\":0,\"gps\":0,\"imu\":0,\"encoder\":" +
                        "0,\"brake_state\":0,\"steering\":0,\"fp_hash\":0,\"logic_notification\":0," +
                        "\"push_bar_camera\":0,\"pose\":0,\"reset\":0,\"state\":0\\}";


        File f = new File(logName);
        if (!f.createNewFile()) {
            fail("Couldn't create log file!");
        }
        FileWriter w = new FileWriter(logName);
        w.write(trace);
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
        messagesReceived = new ArrayList<>();

        for (NodeChannel c : NodeChannel.getLoggingChannels()) {
            subList.add(new Subscriber("PlaybackNodeTest", c.getMsgPath(), (topicName, m) -> {
                messagesReceived.add(m);
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
            if (!messagesExpected.contains(m)) {
                fail("Message " + m.toString() + " was not expected");
            }
            messagesExpected.remove(m);
        }
        for (Message m : messagesExpected) {
            System.err.println("Message " + m.toString() + " was expected but not received");
        }
        if (!messagesExpected.isEmpty()) {
            fail();
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
    public void testPlaybackOnePubLog() {
        fail("Not implemented");
    }

    /**
     * Plays back a standard log file
     */
    @Test
    public void testPlaybackStandardLog() {
        fail("Not implemented");
    }

}