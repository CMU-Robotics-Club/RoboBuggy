package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.serial.SerialConnection;

// spans a new instance of the c++ vision system which will be connected to
// for vision code
public class VisionSystem implements Sensor{

private Process externalProcess;

long lastUpdateTime;
private SensorState currentState;

public boolean reset(){
	//TODO
	return false;
}
	
//spans the vision system thread ( a c++ program which does our image processing)
public VisionSystem(String string){	
	System.out.println("Initializing Vision System");
	try {
		String frontCamIndex_str = Integer.toString(config.FRONT_CAM_INDEX);
		String rearCamIndex_str = Integer.toString(config.REAR_CAM_INDEX);
		externalProcess = new ProcessBuilder(config.VISION_SYSTEM_EXECUTABLE_LOCATION,"-c",frontCamIndex_str,"-c",rearCamIndex_str).start();
	} catch (Exception exc) {
		exc.printStackTrace();
	}
	
}

//TODO connect input stream and output stream of vision system to java 

//returns true if the Visions System was closed properly otherwise return false
public boolean close(){
	System.out.println("closing Vision System");
	externalProcess.destroy();
	return true;
	//TODO
}

public long timeOfLastUpdate(){
	return lastUpdateTime;
}

@Override
public SensorState getState() {
	return currentState;
}

@Override
public boolean isConnected() {
	return false;
}


}
