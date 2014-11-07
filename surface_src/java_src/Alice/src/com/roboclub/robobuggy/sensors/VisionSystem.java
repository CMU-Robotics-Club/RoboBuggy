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

import com.roboclub.robobuggy.main.config;

public class VisionSystem implements Sensor {
	private SensorType sensorType;
	private boolean connected = true;
	
	public VisionSystem(String string) {
		this.sensorType = SensorType.VISION;
		this.connected = false;
		
		try {
			System.loadLibrary("robobuggy_vision");
		} catch (UnsatisfiedLinkError e) {
			System.out.println("Failed to Load Library");
			e.printStackTrace();
			return;
		}
		
		int[] cameras = {config.FRONT_CAM_INDEX, config.REAR_CAM_INDEX};
		String[] labels = {"FRONT", "BACK"};
		
		Thread thread = new Thread(new Runnable() {
			@Override
			public void run() {
				initVisionDefault();
				initVision(cameras, labels, 2);
			}
		});
		thread.start();
	}
	
	/*			JNI Native Methods			*/
	public native int initVision(int[] cameras, String[] labels, int length);
	public native int initVisionDefault();
	public native int disconnect();
	public native int startRecording(String directory);
	public native int stopRecording();

	/*			Callbacks			*/
	public void callback(boolean value) {
		this.connected = value;
	}
	
	public void callback(float x, float y) {
		// TODO update position based on vision
	}
	
	public void callback(float orient) {
		// TODO update orientation based on vision
	}
	
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
		return (disconnect() > 0);
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
		disconnect();

		int[] cameras = {config.FRONT_CAM_INDEX, config.REAR_CAM_INDEX};
		String[] labels = {"FRONT", "BACK"};
		
		Thread thread = new Thread(new Runnable() {
			@Override
			public void run() {
				initVisionDefault();
				initVision(cameras, labels, 2);
			}
		});
		thread.start();
		
		return true;
	}

}
