package com.roboclub.robobuggy.sensors;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.UUID;

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
import com.roboclub.robobuggy.nodes.RBSMNode;
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
	// Returns null if unable to connect, otherwise SerialPort
	private static SerialPort connect(String portName, int baudRate) {
		try {
	        CommPortIdentifier portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
	        
	        if ( portIdentifier.isCurrentlyOwned() ) {
	        	System.err.println("Error: Port currently in use");
	        } else { 
	            CommPort commPort = portIdentifier.open(portName, 2000);
	            
	            if ( commPort instanceof SerialPort ) {
	                SerialPort serialPort = (SerialPort) commPort;
	                serialPort.setSerialPortParams(baudRate,SerialPort.DATABITS_8,SerialPort.STOPBITS_1,SerialPort.PARITY_NONE);
	                return serialPort;
	            }
	        }
		} catch (Exception e) {
			System.err.println("Error: Unable to connect to port" + portName);
		}
		
		return null;
    }
	
	public String newRealSensor(RealNodeEnum nodeType, int baudRate, String port, SensorChannel... sensor) throws Exception {
		// Each sensor will get added to a list of real nodes, returning a UUID.
		//throw an error if unable to connect
		
		//assumes successful connection
		String portKey = UUID.randomUUID().toString();
		
		switch (nodeType) {
		case IMU:
			ImuNode imu = new ImuNode(sensor[0]);
			imu.setSerialPort(connect(port, imu.baudRate()));
			realSensors.put(portKey, imu);
			break;
		case GPS:
			GpsNode gps = new GpsNode(sensor[0]);
			gps.setSerialPort(connect(port, gps.baudRate()));
			realSensors.put(portKey, gps);
			break;
		case RBSM:
			RBSMNode rbsm = new RBSMNode(sensor[0], sensor[1]);
			rbsm.setSerialPort(connect(port, rbsm.baudRate()));
			realSensors.put(portKey, rbsm);
			break;
		default:
			System.out.println("Attempting to add unsupported sensor");
			return null;
		}
		
		return portKey;
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