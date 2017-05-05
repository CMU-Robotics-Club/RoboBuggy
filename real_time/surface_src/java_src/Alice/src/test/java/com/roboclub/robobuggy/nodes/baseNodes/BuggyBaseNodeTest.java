package com.roboclub.robobuggy.nodes.baseNodes;

import com.roboclub.robobuggy.ros.NodeChannel;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static junit.framework.TestCase.assertNotNull;
import static org.junit.Assert.assertEquals;


/**
 * Created by vivaanbahl on 1/5/2017.
 */
public class BuggyBaseNodeTest {

    private BuggyBaseNode node;

    /**
     * Initializes a new BuggyBaseNode
     */
    @Before
    public void setup() {
        node = new BuggyBaseNode(NodeChannel.UNKNOWN_CHANNEL);
    }

    /**
     * Gets rid of the current BuggyBaseNode instance
     */
    @After
    public void teardown() {
        node = null;
    }

    /**
     * Tests that the node is set up properly
     */
    @Test
    public void testNodeInit() {
        assertEquals(node.state, NodeState.NOT_IN_USE);
        assertNotNull(node.timer);
    }

    /**
     * Tests the startNode() method
     */
    @Test
    public void testNodeStart() {
        node.startNode();
        assertEquals(node.state, NodeState.ON);
    }

    /**
     * Tests the shutdown() method of nodes
     */
    @Test
    public void testNodeStop() {
        node.startNode();
        node.shutdown();
        assertEquals(node.state, NodeState.NOT_IN_USE);
    }

    /**
     * Tests the setNodeState() method, with a new state that is valid
     */
    @Test
    public void testSetNodeStateValidState() {
        node.setNodeState(NodeState.ERROR);
        assertEquals(node.state, NodeState.ERROR);
    }

    /**
     * Tests the setNodeState() method, with a new state that is invalid
     */
    @Test
    public void testSetNodeStateInvalidState() {
        node.setNodeState(null);
        assertNotNull(node.state);
    }



}