package com.roboclub.robobuggy.main;

import com.roboclub.robobuggy.robots.AbstractRobot;
import com.roboclub.robobuggy.robots.SimRobot;
import com.roboclub.robobuggy.robots.TransistorDataCollection;
import com.roboclub.robobuggy.serial.RBSerialMessage;
import com.roboclub.robobuggy.simulation.SensorPlayer;
import com.roboclub.robobuggy.ui.Gui;
import com.roboclub.robobuggy.utilities.JNISetup;

import gnu.io.CommPortIdentifier;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;


/** This class is the driver starting up the robobuggy program, if you want the buggy to drive itself you should run this node */
public class RobobuggyMainFile {
    static public AbstractRobot robot;
	
    /*
    public static getRobot(){
    	
    }
    */
    
	
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
   

        List<String> ports = getAvailablePorts();
		//Initialize message headers
		RBSerialMessage.initializeHeaders();

        System.out.println(ports);

        
     	
    	if (RobobuggyConfigFile.DATA_PLAY_BACK) {
            robot = SimRobot.getInstance();//TransistorAuton;//SimRobot.getInstance();
    		//Play back mode enabled
    		new SensorPlayer("logs/2016-03-12-18-15-17/sensors_2016-03-12-18-15-17.txt",1);//"logs/2016-03-12-17-37-04/sensors_2016-03-12-17-37-04.txt", 1);
        }else{
        	robot = TransistorDataCollection.getInstance();
        }
    	
        Gui.getInstance();

        
        	//Play back disabled, create robot
        	robot.startNodes();
			new RobobuggyLogicNotification("Robobuggy Logic Notfication started", RobobuggyMessageLevel.NOTE);

    }
    
    private static List<String> getAvailablePorts() {

        List<String> list = new ArrayList<String>();

        Enumeration<?> portList = CommPortIdentifier.getPortIdentifiers();

        while (portList.hasMoreElements()) {
            CommPortIdentifier portId = (CommPortIdentifier) portList.nextElement();
            if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
                list.add(portId.getName());
            }
        }

        return list;
    }

}