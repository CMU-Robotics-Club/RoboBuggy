package com.roboclub.robobuggy.main;

import gnu.io.CommPortIdentifier;

import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.nodes.RealNodeEnum;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorManager;
import com.roboclub.robobuggy.simulation.SensorPlayer;
import com.roboclub.robobuggy.ui.Gui;


/** This class is the driver starting up the robobuggy program, if you want the buggy to drive itself you should run this node */
public class mainFile {
    static Robot buggy;
    static int num = 0;
    
    public static void main(String args[])  {
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
        
        // Starts the robot
        try {           
        	Robot.getInstance();
            bringup_sim();
        } catch (Exception e) 
        {
            Gui.close();
            System.out.println("Unable to bringup simulated robot. Stacktrace omitted because it's really big.");
            e.printStackTrace();
            return;
        }  
        
        
        if(config.GUI_ON)
        {
            Gui.getInstance();
        }
        
       
    }
    
    //going to start by just connecting to the IMU
    public static void bringup_sim() throws Exception 
    {
        if(config.logging)
        {
            System.out.println("Starting Logging");
            RobotLogger.getInstance();
        }
        
        SensorManager sm = SensorManager.getInstance();
        
        if (config.DATA_PLAY_BACK_DEFAULT) {
        	//initialize a new real sensor with type, port, and channel(s)
        	//sensormanager will (eventually) continue to look on same port for the sensor
        	//returns a key to the new sensor -- remove with this key.
        	String ImuKey = sm.newRealSensor(RealNodeEnum.IMU, config.COM_PORT_IMU, SensorChannel.IMU);
        	String GpsKey = sm.newRealSensor(RealNodeEnum.GPS, config.COM_PORT_GPS_INTEGRATED, SensorChannel.GPS);
        	String RBSMKey = sm.newRealSensor(RealNodeEnum.RBSM, config.COM_PORT_ENCODER, SensorChannel.ENCODER, SensorChannel.STEERING);
        	String LoggingKey = sm.newRealSensor(RealNodeEnum.LOGGING_BUTTON, "", SensorChannel.GUI_LOGGING_BUTTON);
        }
        
        else {
        	SensorPlayer sp = new SensorPlayer("logs/2015-11-14-03-43-56/sensors.txt");
        	new Thread(new Runnable() {
				
				@Override
				public void run() {
					// TODO Auto-generated method stub
					sp.run();
				}
			}).start();
        }
    }
    
    public static List<String> getAvailablePorts() {

        List<String> list = new ArrayList<String>();

        Enumeration portList = CommPortIdentifier.getPortIdentifiers();

        while (portList.hasMoreElements()) {
            CommPortIdentifier portId = (CommPortIdentifier) portList.nextElement();
            if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
                list.add(portId.getName());
            }
        }

        return list;
    }

}
