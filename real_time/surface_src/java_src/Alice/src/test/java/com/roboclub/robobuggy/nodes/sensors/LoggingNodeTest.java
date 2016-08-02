package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import junit.framework.TestCase;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.runners.model.TestClass;

import java.util.ArrayList;

/**
 * Created by vivaanbahl on 7/27/16.
 */
public class LoggingNodeTest extends TestCase {

    private ArrayList<Publisher> pubList;
    private ArrayList<Message> messageList;
    private LoggingNode logger;

    @Before
    public void setUp() throws Exception {
        pubList = new ArrayList<>();
        messageList = new ArrayList<>();

        logger = new LoggingNode("./", NodeChannel.values());
    }

    @After
    public void tearDown() throws Exception {

    }

    /**
     * Logs a file without any incoming messages
     */
    public void testLogEmptyFile() {
        fail("Not implemented");
    }

    /**
     * Logs a file with a single publisher publishing messages
     */
    public void testOnePub() {
        fail("Not implemented");
    }

    /**
     * Logs a file with multiple publishers
     */
    public void testMultiPub() {
        fail("Not implemented");
    }

}