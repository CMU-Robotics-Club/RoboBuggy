package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.ros.Subscriber;
import junit.framework.TestCase;
import org.junit.After;
import org.junit.Before;

import java.util.ArrayList;

/**
 * Created by vivaanbahl on 7/27/16.
 */
public class PlaybackNodeTest extends TestCase {

    private ArrayList<Subscriber> subList;
    private PlaybackNode playbackNode;

    /**
     * Creates a log with the specified trace
     * @param logName name of the log
     * @param trace the formatted String the log will contain
     */
    public void createLogWithTrace(String logName, String trace) {

    }
    
    @Before
    public void setUp() throws Exception {
        subList = new ArrayList<>();
    }

    @After
    public void tearDown() throws Exception {

    }

    /**
     * Plays back an empty log file
     */
    public void testPlaybackEmptyLog() {
        fail("Not implemented");
    }

    /**
     * Plays back a log file with only one sensor
     */
    public void testPlaybackOnePubLog() {
        fail("Not implemented");
    }

    /**
     * Plays back a standard log file
     */
    public void testPlaybackStandardLog() {
        fail("Not implemented");
    }

}