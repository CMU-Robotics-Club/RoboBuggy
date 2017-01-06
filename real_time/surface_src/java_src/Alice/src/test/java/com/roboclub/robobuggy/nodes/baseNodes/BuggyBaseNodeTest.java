package com.roboclub.robobuggy.nodes.baseNodes;

import com.roboclub.robobuggy.ros.NodeChannel;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

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

    @Test
    public void testNodeInit() {
        assertEquals(node.state, NodeState.NOT_IN_USE);
        assertNotNull(node.timer);
    }

    @Test
    public void testNodeStart() {
        node.startNode();
        assertEquals(node.state, NodeState.ON);
    }

    @Test
    public void testNodeStop() {
        node.startNode();
        node.shutdown();
        assertEquals(node.state, NodeState.NOT_IN_USE);
    }

    @Test
    public void testSetNodeStateValidState() {
        node.setNodeState(NodeState.ERROR);
        assertEquals(node.state, NodeState.ERROR);
    }

    @Test
    public void testSetNodeStateInvalidState() {
        node.setNodeState(null);
        assertNotNull(node.state);
    }



}