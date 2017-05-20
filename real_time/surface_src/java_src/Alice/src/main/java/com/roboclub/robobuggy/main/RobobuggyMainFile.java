package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.robots.AbstractRobot;
import com.roboclub.robobuggy.robots.PlayBackRobot;
import com.roboclub.robobuggy.robots.SimRobot;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.utilities.JNISetup;


/**
 * This class is the driver starting up the robobuggy program, if you want the buggy to drive itself you should run this node
 */
public class RobobuggyMainFile {
    private static AbstractRobot robot;

    /**
     * Run Alice
     *
     * @param args : None
     */
    public static void main(String[] args) {
        new RobobuggyLogicNotification("Robobuggy Alice program started", RobobuggyMessageLevel.NOTE);

        try {
            JNISetup.setupJNI(); //must run for jni to install
            //note that errors are just printed to the console since the gui and logging system  has not been created yet
        } catch (NoSuchFieldException | SecurityException | IllegalAccessException | IllegalArgumentException e1) {
            e1.printStackTrace();
        }

        RobobuggyConfigFile.loadConfigFile(); //TODO make sure that logic Notification is setup before this point

        new RobobuggyLogicNotification("Initializing Robot", RobobuggyMessageLevel.NOTE);
        robot = PlayBackRobot.getInstance();

        new RobobuggyLogicNotification("Initializing GUI", RobobuggyMessageLevel.NOTE);
        Gui.getInstance();


        new RobobuggyLogicNotification("Starting Robot", RobobuggyMessageLevel.NOTE);
        robot.startNodes();

    }

    /**
     * Evaluates to a reference to the current Robot
     *
     * @return the robot reference
     */
    public static AbstractRobot getCurrentRobot() {
        return robot;
    }


    /**
     * This method will reset and reload all parameters
     * NOTE this method does not reset the ConfigurationPanel
     *
     * @return
     */
    public static void resetSystem() {
        robot.shutDown();
        //   	Gui.close();
        //   	Gui.getInstance();
//    	robot.getInstance();
        //TODO make this work for real

    }


}
