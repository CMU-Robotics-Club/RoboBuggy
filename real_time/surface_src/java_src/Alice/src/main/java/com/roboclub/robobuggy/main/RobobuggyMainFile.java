package com.roboclub.robobuggy.main;

import gnu.io.CommPortIdentifier;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.simulation.SensorPlayer;
import com.roboclub.robobuggy.ui.Gui;


/** This class is the driver starting up the robobuggy program, if you want the buggy to drive itself you should run this node */
public class RobobuggyMainFile {
    
    /**
	 * Run Alice
	 * @param args :
	 * -g - run without gui
	 * +g - run with gui
	 * -r - run with active false
	 * +r - run with active true
	 */
    public static void main(String[] args)  {
        RobobuggyConfigFile.getInstance();//must be run at least once
        try {
			RobobuggyConfigFile.setupJNI(); //must run for jni to install
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
                RobobuggyConfigFile.GUI_ON = false;
            } 
            else if (args[i].equalsIgnoreCase("+g")) 
            {
                RobobuggyConfigFile.GUI_ON = true;
            } 
            else if (args[i].equalsIgnoreCase("-r")) 
            {
                RobobuggyConfigFile.IS_GUI_CURRENTLY_LOGGING = false;
            } 
            else if (args[i].equalsIgnoreCase("+r")) 
            {
                RobobuggyConfigFile.IS_GUI_CURRENTLY_LOGGING = true;
            }
        }
        
        if(RobobuggyConfigFile.GUI_ON)
        {
            Gui.getInstance();
        }
        
    	if(RobobuggyConfigFile.IS_GUI_CURRENTLY_LOGGING)
        {
            RobotLogger.getInstance();
            //Gui.EnableLogging();
        }
    	
    	if (RobobuggyConfigFile.DATA_PLAY_BACK_DEFAULT) {
    		//Play back mode enabled
    		final SensorPlayer sp = new SensorPlayer("logs/logs/2015-11-15-06-57-21/sensors.txt");
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