package org.roboclub.robobuggy.main;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.util.ArrayList;
import org.roboclub.robobuggy.fauxNodes.FauxEncoderNode;
import org.roboclub.robobuggy.fauxNodes.FauxGPSNode;
import org.roboclub.robobuggy.fauxNodes.FauxIMUNode;
import org.roboclub.robobuggy.fauxNodes.FauxNode;
import org.roboclub.robobuggy.fauxNodes.FauxSteeringNode;
import org.roboclub.robobuggy.logging.RobotLogger;
import org.roboclub.robobuggy.nodes.EncoderNode;
import org.roboclub.robobuggy.nodes.GpsNode;
import org.roboclub.robobuggy.nodes.ImuNode;
import org.roboclub.robobuggy.nodes.SteeringNode;
import org.roboclub.robobuggy.ros.Message;
import org.roboclub.robobuggy.ros.MessageListener;
import org.roboclub.robobuggy.ros.Node;
import org.roboclub.robobuggy.ros.SensorChannel;
import org.roboclub.robobuggy.ros.Subscriber;
import org.roboclub.robobuggy.simulation.FauxRunner;
import org.roboclub.robobuggy.ui.Gui;

/**
 * TODO document
 * @author Trevor Decker
 * @author Kevin B
 * @author Matt Sebeck 
 * @author Vasu
 * @version 0.0
 *
 */
public class mainFile {
	static Robot buggy;

	static int num = 0;
	
	/**
	 * TODO document
	 * @param args
	 */
	public static void main(String args[]) {
		System.out.println("did we get here");
		//ArrayList<Integer> cameras = new ArrayList<Integer>();  //TODO have this set the cameras to use 
		config.getInstance();//must be run at least once
		
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
	
	/**
	 * TODO document 
	 *  Open a serial port
	 * @param portName
	 * @return
	 * @throws Exception
	 */
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
	
	/**
	 * TODO documnet 
	 * @throws Exception
	 */
	public static void bringup_sim() throws Exception {
//		ArrayList<Node> sensorList = new ArrayList<Node>();

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

		ArrayList<FauxNode> fauxSensors = new ArrayList<FauxNode>();
		fauxSensors.add(new FauxIMUNode(SensorChannel.IMU));
		fauxSensors.add(new FauxGPSNode(SensorChannel.GPS));
		fauxSensors.add(new FauxEncoderNode(SensorChannel.ENCODER));
		fauxSensors.add(new FauxSteeringNode(SensorChannel.DRIVE_CTRL));
		String path = "C:\\Users\\Vasu\\Documents\\RoboClub\\RoboBuggy\\offline\\offline_tools\\java_src\\FauxArduino\\logs\\2015-04-12-06-22-37\\sensors.txt";
		new Thread(new FauxRunner(fauxSensors, path)).start();
			
//		LoggingNode ln = new LoggingNode(SensorChannel.IMU.getMsgPath(), "C:\\Users\\Matt\\buggy-log\\run1");
		
//		ImuNode imu = new ImuNode(SensorChannel.IMU);
//		GpsNode gps = new GpsNode(SensorChannel.GPS);
//		EncoderNode enc = new EncoderNode(SensorChannel.ENCODER);
//		SteeringNode drive_ctrl = new SteeringNode(SensorChannel.DRIVE_CTRL);
//		
//		// Set up the IMU
//		SerialPort sp = null;
//		String com = "COM4";//"COM18";
//		try {
//			System.out.println("Initializing IMU Serial Connection");
//			sp = connect(com);
//			System.out.println("IMU connected to " + com);
//		} catch (Exception e) {
//			System.out.println("Unable to connect to necessary device on " + com);
//			e.printStackTrace();
//			throw new Exception("Device not found error");
//		}
//		imu.setSerialPort(sp);
//		sensorList.add(imu);
//
//		// Set up the GPS
//		com = "COM7"; //"COM16";
//		try {
//			System.out.println("Initializing GPS Serial Connection");
//			sp = connect(com);
//			System.out.println("GPS connected to " + com);
//		} catch (Exception e) {
//			System.out.println("Unable to connect to necessary device on " + com);
//			e.printStackTrace();
//			throw new Exception("Device not found error");
//		}
//		gps.setSerialPort(sp);
//		sensorList.add(gps);
	
//		// Set up the Encoder
//		com = "COM14"; //"COM15";
//		try {
//			System.out.println("Initializing ENCODER Serial Connection");
//			sp = connect(com);
//			System.out.println("ENCODER connected to " + com);
//		} catch (Exception e) {
//			System.out.println("Unable to connect to necessary device on " + com);
//			e.printStackTrace();
//			throw new Exception("Device not found error");
//		}
//		enc.setSerialPort(sp);
//		sensorList.add(enc);
	
		new Subscriber(SensorChannel.ENCODER.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				//System.out.println(m.toLogString());
			}
		});

//		// Set up the DRIVE CONTROL
//		com = "COM9"; //"COM17";
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
