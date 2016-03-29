package com.roboclub.robobuggy.main;


import com.roboclub.robobuggy.jetty.gui.JettyServer;
import com.roboclub.robobuggy.robots.AbstractRobot;
import com.roboclub.robobuggy.robots.SimRobot;
import com.roboclub.robobuggy.robots.TransistorAuton;
import com.roboclub.robobuggy.robots.TransistorDataCollection;
import com.roboclub.robobuggy.serial.RBSerialMessage;
import com.roboclub.robobuggy.simulation.SensorPlayer;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.utilities.JNISetup;


/** This class is the driver starting up the robobuggy program, if you want the buggy to drive itself you should run this node */
public class RobobuggyMainFile {
	private static AbstractRobot robot;
    
    /**
	 * Run Alice
	 * @param args : None
	 */
    public static void main(String[] args) {
    	try {
			new JettyServer();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
    	
        try {
			JNISetup.setupJNI(); //must run for jni to install
			//note that errors are just printed to the console since the gui and logging system  has not been created yet
		} catch (NoSuchFieldException e1) {
			e1.printStackTrace();
		} catch (SecurityException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (IllegalArgumentException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (IllegalAccessException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
        
    	RobobuggyConfigFile.loadConfigFile(); //TODO make sure that logic Notification is setup before this point
        robot= TransistorAuton.getInstance();
     	/*
    	if (RobobuggyConfigFile.isDataPlayBack()) {
            robot = SimRobot.getInstance();
        }else{
        	robot = TransistorDataCollection.getInstance();
        }
        */
    	
        Gui.getInstance();

        
        	//Play back disabled, create robot
        	robot.startNodes();
			new RobobuggyLogicNotification("Robobuggy Logic Notfication started", RobobuggyMessageLevel.NOTE);

    }
    
    /**
     * Evaluates to a reference to the current Robot  
     * @return the robot reference 
     */
    public static AbstractRobot getCurrentRobot(){
    	return robot;
    }
    
    
    /**
     * This method will reset and reload all parameters 
     * NOTE this method does not reset the ConfigurationPanel
     * @return 
     */
    public static void resetSystem(){
    	robot.shutDown();
 //   	Gui.close();
 //   	Gui.getInstance();
//    	robot.getInstance();
    	//TODO make this work for real 
    	
    }
    

}