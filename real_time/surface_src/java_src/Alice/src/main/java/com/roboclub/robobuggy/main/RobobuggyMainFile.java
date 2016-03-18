package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.serial.RBSerialMessage;
import com.roboclub.robobuggy.simulation.SensorPlayer;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.utilities.JNISetup;


/** This class is the driver starting up the robobuggy program, if you want the buggy to drive itself you should run this node */
public class RobobuggyMainFile {
    
    /**
	 * Run Alice
	 * @param args : None
	 */
    public static void main(String[] args) {
    	
    	
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

   

		//Initialize message headers
		RBSerialMessage.initializeHeaders();

        Robot.getInstance();
        Gui.getInstance();

        
     	
    	if (RobobuggyConfigFile.isDataPlayBack()) {
    		//Play back mode enabled
    		new SensorPlayer(RobobuggyConfigFile.getPlayBackSourceFile(), 1);

        }
        
        	//Play back disabled, create robot
        	Robot.getInstance().startNodes();
			new RobobuggyLogicNotification("Robobuggy Logic Notfication started", RobobuggyMessageLevel.NOTE);

    }
    
    
    /**
     * This method will reset and reload all parameters 
     * NOTE this method does not reset the ConfigurationPanel
     * @return 
     */
    public static void resetSystem(){
    	Robot.getInstance().shutDown();
 //   	Gui.close();
 //   	Gui.getInstance();
    	Robot.getInstance();
    	//TODO make this work for real 
    	
    }
    

}