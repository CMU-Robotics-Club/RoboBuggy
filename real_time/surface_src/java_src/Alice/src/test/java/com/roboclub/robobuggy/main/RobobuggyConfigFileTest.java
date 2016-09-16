package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.robots.PlayBackRobot;
import com.roboclub.robobuggy.robots.SimRobot;
import com.roboclub.robobuggy.robots.TransistorAuton;
import com.roboclub.robobuggy.robots.TransistorDataCollection;
import junit.framework.TestCase;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import java.io.*;

/**
 * Created by davidneiman on 9/2/16.
 */
public class RobobuggyConfigFileTest {

    @Before
    public void setUp() throws Exception {
        //Runs before each test case, denoted "@Test" before the method
    }

    @After
    public void tearDown() throws Exception {
        //Runs after each test case
    }

    public void createConfigFileWithRobot(String s) throws IOException{
        BufferedWriter writer = new BufferedWriter(new FileWriter(new File("/Users/davidneiman/RoboBuggy/real_time/surface_src/java_src/Alice/config.config")));
        writer.write("{\"COM_PORT_IMU\":\"NOT_SET\",\"COM_PORT_RBSM\":\"NOT_SET\",\"PORT_VISION\":\"NOT_SET\",\"COM_PORT_GPS\":\"NOT_SET\",\"WAYPOINT_SOURCE_LOG_FILE\":\"config.config\",\"PLAY_BACK_SOURCE_FILE\":\"NOT_SET\",\"VIDEO_FILE\":\"/Users/davidneiman/RoboBuggy/real_time/surface_src/java_src/Alice/logs/2016-02-20-06-50-45/webcam.mp4\",\"ROBOT_TYPE\":\"");
        writer.write(s);
        writer.write("\"}");
        writer.close();
    }

    public void createConfigFileWithoutRobot() throws IOException{
        BufferedWriter writer = new BufferedWriter(new FileWriter(new File("/Users/davidneiman/RoboBuggy/real_time/surface_src/java_src/Alice/config.config")));
        writer.write("{\"COM_PORT_IMU\":\"NOT_SET\",\"COM_PORT_RBSM\":\"NOT_SET\",\"PORT_VISION\":\"NOT_SET\",\"COM_PORT_GPS\":\"NOT_SET\",\"WAYPOINT_SOURCE_LOG_FILE\":\"config.config\",\"PLAY_BACK_SOURCE_FILE\":\"NOT_SET\",\"VIDEO_FILE\":\"/Users/davidneiman/RoboBuggy/real_time/surface_src/java_src/Alice/logs/2016-02-20-06-50-45/webcam.mp4\"}");
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
        } catch (IOException e) {
            e.printStackTrace();
            assert false;
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
            assert false;
        }

        //Test getRobotType and setRobotType
        assert RobobuggyConfigFile.getRobotType().equals("TransistorAuton");
        RobobuggyConfigFile.setRobotType("TempBuggy!!!");
        assert RobobuggyConfigFile.getRobotType().equals("TempBuggy!!!");
    }

    /**
     * Creates a config file that specifies a TransistorAuton robot, then creates the robot
     */
    @Test
    public void testTransistorAuton(){
        try {
            createConfigFileWithRobot("TransistorAuton");
            RobobuggyConfigFile.loadConfigFile();
        } catch (IOException e) {
            e.printStackTrace();
            assert false;
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
            assert false;
        }
        assert RobobuggyConfigFile.getRobot() instanceof TransistorAuton;
    }

    /**
     * Creates a config file that specifies a TransistorDataCollection robot, then creates the robot
     */
    @Test
    public void testTransistorDataCollection(){
        try {
            createConfigFileWithRobot("TransistorDataCollection");
            RobobuggyConfigFile.loadConfigFile();
        } catch (IOException e) {
            e.printStackTrace();
            Assert.fail();
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
            Assert.fail();
        }
        assert RobobuggyConfigFile.getRobot() instanceof TransistorDataCollection;
    }

    /**
     * Creates a config file that specifies a PlayBackRobot robot, then creates the robot
     */
    @Test
    public void testPlayBackRobot(){
        try {
            createConfigFileWithRobot("PlayBackRobot");
            RobobuggyConfigFile.loadConfigFile();
        } catch (IOException e) {
            e.printStackTrace();
            Assert.fail();
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
            Assert.fail();
        }
        assert RobobuggyConfigFile.getRobot() instanceof PlayBackRobot;
    }

    /**
     * Creates a config file that specifies a SimRobot robot, then creates the robot
     */
    @Test
    public void testSimRobot(){
        try {
            createConfigFileWithRobot("SimRobot");
            RobobuggyConfigFile.loadConfigFile();
        } catch (IOException e) {
            e.printStackTrace();
            Assert.fail();
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
            Assert.fail();
        }
        assert RobobuggyConfigFile.getRobot() instanceof SimRobot;
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
            //Good;test passed
        }
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
            //Good;test passed
        }
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
            //Good;test passed
        }
    }

    /**
     * Creates a config file without a robot field
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
            //Good;test passed
        }
    }
}