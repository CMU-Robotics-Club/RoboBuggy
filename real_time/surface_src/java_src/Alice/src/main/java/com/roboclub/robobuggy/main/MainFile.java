package com.roboclub.robobuggy.main;

import gnu.io.CommPortIdentifier;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.simulation.SensorPlayer;
import com.roboclub.robobuggy.ui.Gui;


/** This class is the driver starting up the robobuggy program, if you want the buggy to drive itself you should run this node */
public class MainFile {
    
	/**
	 * Run Alice
	 * @param args :
	 * -g - run without gui
	 * +g - run with gui
	 * -r - run with active false
	 * +r - run with active true
	 */
    public static void main(String[] args)  {
        config.getInstance();//must be run at least once
        try {
			config.setupJNI(); //must run for jni to install
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
        System.out.println(ports);
        
        for (int i = 0; i < args.length; i++) {
            if (args[i].equalsIgnoreCase("-g")) {
                config.GUI_ON = false;
            } 
            else if (args[i].equalsIgnoreCase("+g")) 
            {
                config.GUI_ON = true;
            } 
            else if (args[i].equalsIgnoreCase("-r")) 
            {
                config.active = false;
            } 
            else if (args[i].equalsIgnoreCase("+r")) 
            {
                config.active = true;
            }
        }
        
        if(config.GUI_ON)
        {
            Gui.getInstance();
        }
        
    	if(config.logging)
        {
            RobotLogger.getInstance();
            Gui.EnableLogging();
        }
    	
    	if (config.DATA_PLAY_BACK_DEFAULT) {
    		//Play back mode enabled
    		SensorPlayer sp = new SensorPlayer("logs/2015-11-21-07-08-10/sensors.txt");
        	new Thread(new Runnable() {
				
				@Override
				public void run() {
					// TODO Auto-generated method stub
					sp.run();
				}
			}).start();
        }
        else {
        	//Play back disabled, create robot
        	Robot.getInstance().startNodes();
        }
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
