package com.roboclub.robobuggy.main;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Created by davidneiman on 9/2/16.
 */
public class RobobuggyConfigFileTest {

    /**
     * Sets up test cases
     *  @throws Exception
     */
    @Before
    public void setUp() {
        //Runs before each test case, denoted "@Test" before the method

        //Grabbed the initial code from my standard test and just copy-pasted it here
        //This should reset the configuration state between tests

        //Create a new config file with a reasonable robot type
        try {
            createConfigFileWithRobot("TransistorAuton");
            RobobuggyConfigFile.loadConfigFile();
        } catch (IOException|IllegalArgumentException e) {
            e.printStackTrace();
            Assert.fail();
        }
    }

    /**
     * Cleans up test cases
     * @throws Exception
     */
    @After
    public void tearDown() {
        //Runs after each test case
    }

    /**
     * Creates a config file with the specified robot name
     * @param robotName name of the robot to create (e.g. "TransistorAuton", "SimRobot")
     * @throws IOException if we couldn't create the config file
     */
    public void createConfigFileWithRobot(String robotName) throws IOException {
        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(new File("config.config")), "UTF-8"));

        writer.write("{" +
                "\"COM_PORT_IMU\":\"NOT_SET\"," +
                "\"COM_PORT_RBSM\":\"NOT_SET\"," +
                "\"PORT_VISION\":\"NOT_SET\"," +
                "\"COM_PORT_GPS\":\"NOT_SET\"," +
                "\"WAYPOINT_SOURCE_LOG_FILE\":\"config.config\"," +
                "\"PLAY_BACK_SOURCE_FILE\":\"NOT_SET\"," +
                "\"VIDEO_FILE\":\"NOT_SET\"," +
                "\"ROBOT_TYPE\":\"");
        writer.write(robotName);
        writer.write("\"}");
        writer.close();
    }

    /**
     * Creates a config file without a ROBOT field
     * Use only for testing; it's supposed to lead to null-pointer exceptions
     *
     * @throws IOException if we couldn't create the config file
     */
    public void createConfigFileWithoutRobot() throws IOException {
        BufferedWriter writer = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(new File("config.config")), "UTF-8"));

        writer.write("{" +
                "\"COM_PORT_IMU\":\"NOT_SET\"," +
                "\"COM_PORT_RBSM\":\"NOT_SET\"," +
                "\"PORT_VISION\":\"NOT_SET\"," +
                "\"COM_PORT_GPS\":\"NOT_SET\"," +
                "\"WAYPOINT_SOURCE_LOG_FILE\":\"config.config\"," +
                "\"PLAY_BACK_SOURCE_FILE\":\"NOT_SET\"," +
                "\"VIDEO_FILE\":\"NOT_SET" +
                "\"}");
        writer.close();
    }

    //I'd like to test reading in from a config file, but I'm not sure what I'm actually reading in

    /**
     * Tests basic get/set functionality for a normal config file
     * Doesn't actually create any robots
     */
    @Test
    public void testRetention() {

        //Create a new config file with a reasonable robot type
        try {
            createConfigFileWithRobot("TransistorAuton");
            RobobuggyConfigFile.loadConfigFile();
        } catch (IOException|IllegalArgumentException e) {
            e.printStackTrace();
            Assert.fail();
        }

        //Create a bunch of test strings
        String[] teststring = new String[6];
        for(int x = 0; x < 6; x++){
            String temp = "";
            for(int y = 0; y < x; y++){
                temp += "S";
            }
            temp += "afetyDuck!";
            teststring[x] = temp;
        }

        //Test getRobotType
        assertEquals(RobobuggyConfigFile.getRobotType(), "TransistorAuton");

        //Test other stuff
        RobobuggyConfigFile.setImuEnabled(false);
        assertFalse(RobobuggyConfigFile.isImuEnabled());
        RobobuggyConfigFile.setImuEnabled(true);
        assertTrue(RobobuggyConfigFile.isImuEnabled());

        RobobuggyConfigFile.setComPortImu(teststring[0]);
        assertEquals(RobobuggyConfigFile.getComPortImu(), teststring[0]);

        RobobuggyConfigFile.setConfigFile("config.config");
        assertEquals(RobobuggyConfigFile.getConfigFile(), "config.config");

        RobobuggyConfigFile.setWayPointSourceLogFile(teststring[1]);
        assertEquals(RobobuggyConfigFile.getWaypointSourceLogFile(), teststring[1]);

        RobobuggyConfigFile.setPlayBackSourceFile(teststring[2]);
        assertEquals(RobobuggyConfigFile.getPlayBackSourceFile(), teststring[2]);

        RobobuggyConfigFile.setComPortGps(teststring[3]);
        assertEquals(RobobuggyConfigFile.getComPortGPS(), teststring[3]);

        RobobuggyConfigFile.setComPortRBSM(teststring[4]);
        assertEquals(RobobuggyConfigFile.getComPortRBSM(), teststring[4]);

        RobobuggyConfigFile.setGpsEnabled(false);
        assertFalse(RobobuggyConfigFile.isGpsEnabled());
        RobobuggyConfigFile.setGpsEnabled(true);
        assertTrue(RobobuggyConfigFile.isGpsEnabled());

        RobobuggyConfigFile.setEncoderEnabled(false);
        assertFalse(RobobuggyConfigFile.isEncoderEnabled());
        RobobuggyConfigFile.setEncoderEnabled(true);
        assertTrue(RobobuggyConfigFile.isEncoderEnabled());

        RobobuggyConfigFile.setPortVision(teststring[5]);
        assertEquals(RobobuggyConfigFile.getPortVision(), teststring[5]);

        RobobuggyConfigFile.setVisionSystemEnabled(false);
        assertFalse(RobobuggyConfigFile.isVisionSystemEnabled());
        RobobuggyConfigFile.setVisionSystemEnabled(true);
        assertTrue(RobobuggyConfigFile.isVisionSystemEnabled());

        double playbackSpeed = Math.PI;
        RobobuggyConfigFile.setPlayBackSpeed(playbackSpeed);
        assertEquals(RobobuggyConfigFile.getPlayBackSpeed(), playbackSpeed, 0.0);
    }

    /**
     * Creates all the different robot types and tests to make sure they're the correct types
     */
    @Test
    public void testAllRobotTypes(){
        String[] robots = {"TransistorAuton", "TransistorDataCollection", "PlayBackRobot", "SimRobot"};
        for(int x = 0; x < robots.length; x++){
            try {
                createConfigFileWithRobot(robots[x]);
                RobobuggyConfigFile.loadConfigFile();
            } catch (IOException | IllegalArgumentException e) {
                e.printStackTrace();
                Assert.fail();
            }
            assert(RobobuggyConfigFile.getRobot().getClass().getSimpleName().equals(robots[x]));
        }

    }

    /**
     * Creates a config file with a robot field but no robot name
     * It's expected to throw an IllegalArgumentException
     */
    @Test
    public void testNoName(){
        try {
            createConfigFileWithRobot("");
            RobobuggyConfigFile.loadConfigFile();
            Assert.fail();
        } catch (IOException e) {
            e.printStackTrace();
            Assert.fail();
        } catch (IllegalArgumentException e) {
            return;
        }
        Assert.fail();
    }

    /**
     * Creates a config file that specifies a robot but gives an invalid name
     * It's expected to throw an IllegalArgumentException
     */
    @Test
    public void testRandomName(){
        try {
            createConfigFileWithRobot("SafetyDuck!!!");
            RobobuggyConfigFile.loadConfigFile();
            Assert.fail();
        } catch (IOException e) {
            e.printStackTrace();
            Assert.fail();
        } catch (IllegalArgumentException e) {
            return;
        }
        Assert.fail();
    }

    /**
     * Creates a config file with the robot field "NOT_SET"
     * It's expected to throw an IllegalArgumentException
     */
    @Test
    public void testNotSet(){
        try {
            createConfigFileWithRobot("NOT_SET");
            RobobuggyConfigFile.loadConfigFile();
            Assert.fail();
        } catch (IOException e) {
            e.printStackTrace();
            Assert.fail();
        } catch (IllegalArgumentException e) {
            return;
        }
        Assert.fail();
    }

    /**
     * Creates a config file without a key-robot value pair
     * This is different from the above because the JSON parser shouldn't find a robot field
     * It's expected to throw a NullPointerException (not an IllegalArgument exception like the previous)
     */
    @Test
    public void testNoRobot(){
        try {
            createConfigFileWithoutRobot();
            RobobuggyConfigFile.loadConfigFile();
            Assert.fail();
        } catch (IOException | IllegalArgumentException e) {
            e.printStackTrace();
            Assert.fail();
        } catch (NullPointerException e) {
            return;
        }
        Assert.fail();
    }
}