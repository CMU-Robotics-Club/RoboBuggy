package com.roboclub.robobuggy.sensors;

/**
 * 
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;

import com.roboclub.robobuggy.main.config;

// spans a new instance of the c++ vision system which will be connected to
// for vision code
public class VisionSystem implements Sensor{
private SensorType thisSensorType;
private long lastUpdateTime;
private SensorState currentState;
private BufferedInputStream input;
private BufferedOutputStream output;

public boolean reset(){
	//TODO
	return false;
}
	
//spans the vision system thread ( a c++ program which does our image processing)
public VisionSystem(String string){
	System.out.println("Initializing Vision System");
	try {
		//System.loadLibrary("robo_vison");
		
		String frontCamIndex_str = Integer.toString(config.FRONT_CAM_INDEX);
		String rearCamIndex_str = Integer.toString(config.REAR_CAM_INDEX);
		System.out.println("vision attempt start");
		Process externalProcess = new ProcessBuilder(config.VISION_SYSTEM_EXECUTABLE_LOCATION,"-c",frontCamIndex_str,"-c",rearCamIndex_str).start();
		//Process externalProcess = new ProcessBuilder(config.VISION_SYSTEM_EXECUTABLE_LOCATION,"-c",frontCamIndex_str).start();
		
		input = new BufferedInputStream(externalProcess.getInputStream());
		output = new BufferedOutputStream(externalProcess.getOutputStream());
	} catch (Exception exc) {
		exc.printStackTrace();
	}
	
	this.thisSensorType = SensorType.VISION;
}

//TODO connect input stream and output stream of vision system to java 
public void write(String data) {
	try {
		output.write(data.getBytes());
	} catch (IOException e) {
		e.printStackTrace();
	}
}

//returns true if the Visions System was closed properly otherwise return false
public boolean close(){
	System.out.println("closing Vision System");
	//externalProcess.destroy();
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

@Override
public SensorType getSensorType() {
	return thisSensorType;
}

@Override
public void publish() {
	// TODO Auto-generated method stub
	
}

}
