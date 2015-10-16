package com.roboclub.robobuggy.sensors;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import com.roboclub.robobuggy.calculatedNodes.BaseCalculatedNode;
import com.roboclub.robobuggy.calculatedNodes.CalculatedEncoderNode;
import com.roboclub.robobuggy.calculatedNodes.CalculatedGPSNode;
import com.roboclub.robobuggy.calculatedNodes.CalculatedIMUNode;
import com.roboclub.robobuggy.calculatedNodes.CalculatedNodeEnum;
import com.roboclub.robobuggy.calculatedNodes.NodeCalculator;
import com.roboclub.robobuggy.fauxNodes.FauxEncoderNode;
import com.roboclub.robobuggy.fauxNodes.FauxGPSNode;
import com.roboclub.robobuggy.fauxNodes.FauxIMUNode;
import com.roboclub.robobuggy.fauxNodes.FauxNode;
import com.roboclub.robobuggy.fauxNodes.FauxSteeringNode;
//import com.roboclub.robobuggy.nodes.EncoderNode;
import com.roboclub.robobuggy.nodes.GpsNode;
import com.roboclub.robobuggy.nodes.ImuNode;
import com.roboclub.robobuggy.nodes.RealNodeEnum;
//import com.roboclub.robobuggy.nodes.SteeringNode;
import com.roboclub.robobuggy.ros.Node;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.simulation.FauxRunner;

public class SensorManager {
	
	//Need to manage real, simulated, and calculated sensors
	
	private LinkedHashMap<String, Node> realSensors;
	private LinkedHashMap<String, LinkedHashMap<SensorChannel, FauxNode>> simulatedSensors;
	private LinkedHashMap<String, BaseCalculatedNode> calculatedSensors;
	private static SensorManager sm;
	
	private SensorManager() {
		realSensors = new LinkedHashMap<String, Node>();
		simulatedSensors = new LinkedHashMap<String, LinkedHashMap<SensorChannel, FauxNode>>();
		calculatedSensors = new LinkedHashMap<String, BaseCalculatedNode>();	
	}

	// Open a serial port
	private static SerialPort connect(String portName) throws Exception
    {
        CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
        if ( portIdentifier.isCurrentlyOwned() ) {
            System.out.println("Error: Port is currently in use");
            return null;
        } else {
        	//TODO fix this so that it is not potato 
            CommPort commPort = portIdentifier.open("potato", 2000);
            
            if ( commPort instanceof SerialPort ) {
                SerialPort serialPort = (SerialPort) commPort;
                serialPort.setSerialPortParams(57600,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
                return serialPort;
            } else {
                System.out.println("Error: Only serial ports are handled by this example.");
            }
        }
		return null;
    }	
	
	//TODO: Come up with a better way to determine which sensor to create
	public void newRealSensor(RealNodeEnum node, SensorChannel sensor, String port) throws Exception {
		if (realSensors.containsKey(port)) {
			throw new Exception("Trying to connect on a port which exists already!");
		}
		
		SerialPort sp = null;
		try {
			System.out.println("Initializing serial connection on port " + port);
			sp = connect(port);
			System.out.println("Connected on port " + port);
		} catch (Exception  e) {
			System.out.println("Unable to connect to " + sensor.toString() + " on port " + port);
			e.printStackTrace();
			throw new Exception("Device not found error");
		}

		switch (node) {
		case IMU:
			ImuNode imu = new ImuNode(sensor);
			imu.setSerialPort(sp);
			realSensors.put(port, imu);
			break;
		case GPS:
			GpsNode gps = new GpsNode(sensor);
			gps.setSerialPort(sp);
			realSensors.put(port, gps);			
			break;
//disabling this temporarily because I need to figure out how to handle multiple channels
//		case ENCODER:
//			EncoderNode enc = new EncoderNode(sensor);
//			enc.setSerialPort(sp);
//			realSensors.put(port, enc);
//			break;
//		case DRIVE_CTRL:
//			SteeringNode steer = new SteeringNode(sensor);
//			steer.setSerialPort(sp);
//			realSensors.put(port, steer);
//			break;			
		default:
			System.out.println("Invalid Sensor Type");
			throw new Exception("Stop trying to initialize sensors that don't exist!");
		}
	}
	
	//Due to how I wrote the code, you'll need to initialize all of the sensors you want at once.
	//Disable the ones you don't want later if you want, I guess?
	//TODO: go and fix this
	public void newFauxSensors(String path, SensorChannel... sensors) {
		//A pretty terrible solution for now, this will assume that there will only be one type of
		//simulated sensor per log file. This is an okay assumption for the data we are currently
		//outputting, but if we change the log file to log multiple sensors with the granularity
		//we are doing right now, this will need some updating
		LinkedHashMap<SensorChannel, FauxNode> fauxSensors = new LinkedHashMap<SensorChannel, FauxNode>();
		for (SensorChannel sensor : sensors) {
			switch (sensor) {
			case IMU:
				fauxSensors.put(sensor, new FauxIMUNode(sensor));
				break;
			case GPS:
				fauxSensors.put(sensor, new FauxGPSNode(sensor));
				break;
			case ENCODER:
				fauxSensors.put(sensor, new FauxEncoderNode(sensor));
				break;
			case DRIVE_CTRL:
				fauxSensors.put(sensor, new FauxSteeringNode(sensor));
				break;
			default:
				break;
			}
		}
		new Thread(new FauxRunner(new ArrayList<FauxNode>(fauxSensors.values()), path)).start();
		simulatedSensors.put(path, fauxSensors);
	}
	
	// I believe this will only allow for a single calculated sensor per type. This may need to be updated in the future
	public void newCalculatedSensor(CalculatedNodeEnum node, SensorChannel sensor, NodeCalculator calc, int delay) {
		switch(node) {
		case IMU:
			CalculatedIMUNode imu = new CalculatedIMUNode(sensor, calc, delay);
			calculatedSensors.put(calc.getClass().getName(), imu);
			imu.crunch();
			break;
		case GPS:
			CalculatedGPSNode gps = new CalculatedGPSNode (sensor, calc, delay);
			calculatedSensors.put(calc.getClass().getName(), gps);
			gps.crunch();
			break;
		case ENCODER:
			CalculatedEncoderNode enc = new CalculatedEncoderNode(sensor, calc, delay);
			calculatedSensors.put(calc.getClass().getName(), enc);
			enc.crunch();
			break;
		default:
			break;
		}
	}
	
	public void disableFauxNode(String path, SensorChannel sensor) {
		if (simulatedSensors.containsKey(path) && simulatedSensors.get(path).containsKey(sensor)) {
			simulatedSensors.get(path).get(sensor).disable();
		} else {
			System.out.println("Trying to disable a Faux node on an invalid path / sensor");
		}
	}
	
	public void enableFauxNode(String path, SensorChannel sensor) {
		if (simulatedSensors.containsKey(path) && simulatedSensors.get(path).containsKey(sensor)) {
			simulatedSensors.get(path).get(sensor).enable();
		} else {
			System.out.println("Trying to enable a Faux node on an invalid path / sensor");
		}
	}
	
	public static SensorManager getInstance() {
		if (sm == null) {
			sm = new SensorManager();
		}
		return sm;
	}
}