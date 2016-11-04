package com.roboclub.robobuggy.nodes.sensors;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Created by vivaanbahl on 9/8/16.
 */
public class HillcrestNodeTest {

    @Before
    public void setUp() throws Exception {
    }

    @After
    public void tearDown() throws Exception {

    }

    /**
     * Tests that we can safely create an instance of the node
     *
     * No input arguments
     * Expects no output or errors
     */
    @Test
    public void testCreation() {
        HillcrestNode imu = new HillcrestNode();
        assertEquals(imu.getName(), "Hillcrest IMU");
    }

}