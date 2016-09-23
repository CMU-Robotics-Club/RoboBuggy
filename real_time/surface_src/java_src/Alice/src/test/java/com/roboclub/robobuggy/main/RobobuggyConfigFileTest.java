package com.roboclub.robobuggy.main;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.io.*;

/**
 * Created by davidneiman on 9/2/16.
 */
public class RobobuggyConfigFileTest {

    /**
     * Sets up test cases
     *  @throws Exception
     */
    @Before
    public void setUp() throws Exception {
        //Runs before each test case, denoted "@Test" before the method
    }

    /**
     * Cleans up test cases
     * @throws Exception
     */
    @After
    public void tearDown() throws Exception {
        //Runs after each test case
    }

    /**
     * Creates a config file, setting the ROBOT field to the given input
     * @param s: String to be used as the robot type
     * @throws IOException
     */
    public void createConfigFileWithRobot(String s) throws IOException{
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
        writer.write(s);
        writer.write("\"}");
        writer.close();
    }

    /**
     * Creates a config file without a ROBOT field
     * Use only for testing; it's supposed to lead to null-pointer exceptions
     * @throws IOException
     */
    public void createConfigFileWithoutRobot() throws IOException{
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


    /**
     * Tests basic get/set functionality for a normal config file
     * Doesn't actually create any robots
     */
    @Test
    public void testRobotTypeGetSet() {

        //Create a new config file with a reasonable robot type
        try {
            createConfigFileWithRobot("TransistorAuton");
            RobobuggyConfigFile.loadConfigFile();
        } catch (IOException|IllegalArgumentException e) {
            e.printStackTrace();
            Assert.fail();
        }

        //Test getRobotType
        assert(RobobuggyConfigFile.getRobotType().equals("TransistorAuton"));
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
            } catch (IOException e) {
                e.printStackTrace();
                Assert.fail();
            } catch (IllegalArgumentException e) {
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
        } catch (IOException e) {
            e.printStackTrace();
            Assert.fail();
        }catch (IllegalArgumentException e) {
            e.printStackTrace();
            Assert.fail();
        } catch (NullPointerException e) {
            return;
        }
        Assert.fail();
    }
}