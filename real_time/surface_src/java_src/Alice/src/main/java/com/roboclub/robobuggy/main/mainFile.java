package com.roboclub.robobuggy.main;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.nio.file.Paths;
import java.util.ArrayList;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.nodes.EncoderNode;
import com.roboclub.robobuggy.nodes.GpsNode;
import com.roboclub.robobuggy.nodes.ImuNode;
import com.roboclub.robobuggy.nodes.SteeringNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.Gui;

public class mainFile {
	static Robot buggy;
	static int num = 0;
	
	public static void main(String args[]) {
		//ArrayList<Integer> cameras = new ArrayList<Integer>();  //TODO have this set the cameras to use 
		config.getInstance();//must be run at least once
		
		System.out.println(Paths.get("").toAbsolutePath().toString());
		System.err.println(Paths.get("").toAbsolutePath().toString());
		
		for (int i = 0; i < args.length; i++) {
			if (args[i].equalsIgnoreCase("-g")) {
				config.GUI_ON = false;
			} else if (args[i].equalsIgnoreCase("+g")) {
				config.GUI_ON = true;
			} else if (args[i].equalsIgnoreCase("-r")) {
				config.active = false;
			} else if (args[i].equalsIgnoreCase("+r")) {
				config.active = true;
			}
		}
		
		if(config.GUI_ON){
			Gui.getInstance();
		}
		
		// Starts the robot
		if(config.DATA_PLAY_BACK_DEFAULT){
			try {
				bringup_sim();
			} catch (Exception e) {
				Gui.close();
				System.out.println("Unable to bringup simulated robot. Stacktrace omitted because it's really big.");
				e.printStackTrace();
				return;
			}
		} else {
			Robot.getInstance();
		}	
	}
	
	// Open a serial port
	private static SerialPort connect(String portName) throws Exception
    {
        CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
        if ( portIdentifier.isCurrentlyOwned() )
        {
            System.out.println("Error: Port is currently in use");
            return null;
        }
        else
        {
        	//TODO fix this so that it is not potato 
            CommPort commPort = portIdentifier.open("potato", 2000);
            
            if ( commPort instanceof SerialPort )
            {
                SerialPort serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(57600,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
                return serialPort;
            }
            else
            {
                System.out.println("Error: Only serial ports are handled by this example.");
            }
        }
		return null;
    }	
	public static void bringup_sim() throws Exception {
		ArrayList<Node> sensorList = new ArrayList<Node>();

		// Turn on logger!
		if(config.logging){
			System.out.println("Starting Logging");
			RobotLogger.getInstance();
		}

		// Initialize Sensor
		/*if (config.GPS_DEFAULT) {
			System.out.println("Initializing GPS Serial Connection");
			FauxGps gps = new FauxGps(SensorChannel.GPS);
			sensorList.add(gps);

		}*/

		Gui.EnableLogging();

	
//		LoggingNode ln = new LoggingNode(SensorChannel.IMU.getMsgPath(), "C:\\Users\\Matt\\buggy-log\\run1");
		
		ImuNode imu = new ImuNode(SensorChannel.IMU);
		GpsNode gps = new GpsNode(SensorChannel.GPS);
		EncoderNode enc = new EncoderNode(SensorChannel.ENCODER,SensorChannel.STEERING);
		SteeringNode drive_ctrl = new SteeringNode(SensorChannel.DRIVE_CTRL);
		
		// Set up the IMU
		SerialPort sp = null;
		String com = config.COM_PORT_IMU;
		try {
			System.out.println("Initializing IMU Serial Connection");
			sp = connect(com);
			System.out.println("IMU connected to " + com);
		} catch (Exception e) {
			System.out.println("Unable to connect to necessary device on " + com);
			e.printStackTrace();
			throw new Exception("Device not found error");
		}
		imu.setSerialPort(sp);
		sensorList.add(imu);

		// Set up the GPS
		com = config.COM_PORT_GPS_INTEGRATED;
		try {
			System.out.println("Initializing GPS Serial Connection");
			sp = connect(com);
			System.out.println("GPS connected to " + com);
		} catch (Exception e) {
			System.out.println("Unable to connect to necessary device on " + com);
			e.printStackTrace();
			throw new Exception("Device not found error");
		}
		gps.setSerialPort(sp);
		sensorList.add(gps);
	
		// Set up the Encoder
		com = config.COM_PORT_ENCODER;
		try {
			System.out.println("Initializing ENCODER Serial Connection");
			sp = connect(com);
			System.out.println("ENCODER connected to " + com);
		} catch (Exception e) {
			System.out.println("Unable to connect to necessary device on " + com);
			e.printStackTrace();
			throw new Exception("Device not found error");
		}
		enc.setSerialPort(sp);
		sensorList.add(enc);
	
		new Subscriber(SensorChannel.ENCODER.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				//System.out.println(m.toLogString());
			}
		});

		// Set up the DRIVE CONTROL
//		com = config.COM_PORT_DRIVE_CONTROL;
//		try {
//			System.out.println("Initializing DRIVE CONTROL Serial Connection");
//			sp = connect(com);
//			System.out.println("DRIVE CONTROL connected to " + com);
//		} catch (Exception e) {
//			System.out.println("Unable to connect to necessary device on " + com);
//			e.printStackTrace();
//			throw new Exception("Device not found error");
//		}
//		drive_ctrl.setSerialPort(sp);
//		sensorList.add(drive_ctrl);
	}
}
