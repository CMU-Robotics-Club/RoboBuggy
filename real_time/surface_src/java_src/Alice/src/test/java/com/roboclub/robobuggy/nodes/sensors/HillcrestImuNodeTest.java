package com.roboclub.robobuggy.nodes.sensors;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Created by vivaanbahl on 9/8/16.
 */
public class HillCrestImuNodeTest {

    private HillCrestImuNode testNode;

    /**
     * setup done before each test
     */
    @Before
    public void setUp() {
        testNode = new HillCrestImuNode();
    }

    /**
     * work that's done after each test
     */
    @After
    public void tearDown() {
    }

    /**
     * Tests that we can safely create an instance of the node
     *
     * No input arguments
     * Expects no output or errors
     */
    @Test
    public void testCreation() {
        HillCrestImuNode imu = new HillCrestImuNode();
        assertEquals(imu.getName(), "Hillcrest IMU");
    }

    /**
     * Tests that we can convert fixed point numbers to float
     *
     * Inputs a set of fixed point numbers expressed as integers
     * Expects the "correctly" converted numbers - the fixed point system
     * has a resolution of 2^(-n)
     *
     * No errors should be thrown
     */
    @Test
    public void testConvertQn() {
        int q1 = 53;
        int n1 = 2;
        double expected1 = 13.37;
        assertEquals(expected1, testNode.convertQNToDouble(q1, n1), 0.25);

        int q2 = 12868;
        int n2 = 12;
        double expected2 = 3.1417;
        assertEquals(expected2, testNode.convertQNToDouble(q2, n2), 0.0002);

        int q3 = 87;
        int n3 = 5;
        double expected3 = 2.7181;
        assertEquals(expected3, testNode.convertQNToDouble(q3, n3), 0.0312);

        int q4 = 69388;
        int n4 = 14;
        double expected4 = 4.2351;
        assertEquals(expected4, testNode.convertQNToDouble(q4, n4), 0.00006);

        int q5 = 3900;
        int n5 = 8;
        double expected5 = 15.2352;
        assertEquals(expected5, testNode.convertQNToDouble(q5, n5), 0.0039);

        int q6 = 0;
        int n6 = 10;
        double expected6 = 0.0;
        assertEquals(expected6, testNode.convertQNToDouble(q6, n6), 0.0);

        // int max for q14
        int q7 = 0x7FFF;
        int n7 = 1;
        double expected7 = 16383.5;
        assertEquals(expected7, testNode.convertQNToDouble(q7, n7), 0.0);

        // int min for q14
        int q8 = 0x8000;
        int n8 = 1;
        double expected8 = -16384.0;
        assertEquals(expected8, testNode.convertQNToDouble(q8, n8), 0.0);


    }


}