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

import java.io.File;
import java.io.IOException;

import com.roboclub.robobuggy.main.config;

public class VisionSystem implements Sensor {
	private SensorType sensorType;
	private boolean connected = true;
	
	// spans the vision system thread ( a c++ program which does our image
	// processing)
	public VisionSystem(String string) {
		this.sensorType = SensorType.VISION;
		this.connected = false;
		
		// TODO implement init from 
		String frontCamIndex_str = Integer.toString(config.FRONT_CAM_INDEX);
		String rearCamIndex_str = Integer.toString(config.REAR_CAM_INDEX);
		
		try {
			System.loadLibrary("robobuggy_vision");
		} catch (UnsatisfiedLinkError e) {
			System.out.println("Failed to Load Library");
			e.printStackTrace();
			return;
		}
		
		Thread thread = new Thread(new Runnable() {
			@Override
			public void run() {
				initVisionDefault();
			}
		});
		thread.start();
		
		connected = true;
	}
	
	/*			JNI Native Methods			*/
	public native int initVision(int[] cameras, String[] labels, int length);
	public native int initVisionDefault();
	public native int disconnect();
	public native int startRecording(String directory);
	public native int stopRecording();

	/*			Inherited Methods from Sensor		*/
	@Override
	public SensorState getState() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean isConnected() {
		return this.connected;
	}

	@Override
	public long timeOfLastUpdate() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public boolean close() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public SensorType getSensorType() {
		return this.sensorType;
	}

	@Override
	public void publish() {
		// TODO Auto-generated method stub

	}

	@Override
	public boolean reset() {
		// TODO Auto-generated method stub
		return false;
	}

}
