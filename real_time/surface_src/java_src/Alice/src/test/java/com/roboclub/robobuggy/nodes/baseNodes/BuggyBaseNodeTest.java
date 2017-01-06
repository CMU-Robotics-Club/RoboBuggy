package com.roboclub.robobuggy.nodes.baseNodes;

import com.roboclub.robobuggy.ros.NodeChannel;
import org.junit.After;
import org.junit.Before;

import static org.junit.Assert.*;

/**
 * Created by vivaanbahl on 1/5/2017.
 */
public class BuggyBaseNodeTest {

    BuggyBaseNode node;

    @Before
    public void setup() {
        node = new BuggyBaseNode(NodeChannel.UNKNOWN_CHANNEL);
    }

    @After
    public void teardown() {
        node = null;
    }
}