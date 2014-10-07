package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.serial.SerialConnection;

// spans a new instance of the c++ vision system which will be connected to
// for vision code
public class VisionSystem implements Sensor{

//spans the vision system thread ( a c++ program which does our image processing)
public VisionSystem(String string){	
	System.out.println("Initializing Vision System");
	try {
		String frontCamIndex_str = Integer.toString(config.FRONT_CAM_INDEX);
		String rearCamIndex_str = Integer.toString(config.REAR_CAM_INDEX);
		new ProcessBuilder(config.VISION_SYSTEM_EXECUTABLE_LOCATION,"-c",frontCamIndex_str,"-c",rearCamIndex_str).start();
	} catch (Exception exc) {
		exc.printStackTrace();
	}
	
}

//returns true if the Visions System was closed properly otherwise return false
public boolean close(){
	return false;
	//TODO
}

}
