package com.roboclub.robobuggy.sensors;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;

/**
 * 
 * @author Matt Sebek
 *
 * @version 0.5
 * 
 *          CHANGELOG: NONE
 * 
 *          DESCRIPTION: TODO
 */

public class FauxGps implements Sensor {

	private long lastUpdateTime;
	private SensorState currState;
	private SensorType sensorType;
	
	private Publisher msgPub;
	private Publisher statePub;

	// TODO change arguments to paths only?
	public FauxGps(SensorChannel sensor) {
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		sensorType = SensorType.GPS;

		statePub.publish(new StateMessage(SensorState.ON));

		// TODO this is wrong
		lastUpdateTime = -1;
		
		// Start the actual publishing thread.
		new Thread(new gpsPinger()).start();
	}

	public long timeOfLastUpdate() {
		return lastUpdateTime;
	}

	@Override
	public SensorState getState() {
		/*if (System.currentTimeMillis() - lastUpdateTime > SENSOR_TIME_OUT) {
			currState = SensorState.DISCONNECTED;
			this.currState = SensorState.ERROR;
			statePub.publish(new StateMessage(this.currState));
		}*/

		return currState;
	}

	@Override
	public SensorType getSensorType() {
		return sensorType;
	}

	private class gpsPinger implements Runnable {
		// TODO have global state that we retrieve, so we can test stuff with simulation
		private double latitude = 0.0;
		private double longitude = 0.0;
		
		public gpsPinger() {
			float latitude = 0, longitude = 0;
			int state = 0;
		
		}
		
		@Override
		public void run() {
			// TODO check whether or not we should continue
			while(true) {
				try {
					// Send a message at 60hz
					Thread.sleep(1000 / 60);
				} catch (InterruptedException ie) {
					throw new RuntimeException(
							"Sleep should not be throwing");
				}
				msgPub.publish(new GpsMeasurement(latitude, longitude));
			}
		}
	}


	@Override
	public boolean isConnected() {
		// Return false if we want to do some kind of reliability test.
		return true;
	}

	@Override
	public boolean close() {
		// Destroy....nothing, i guess?
		return true;
	}

	@Override
	public void publish() {
		System.out.println("THIS SHOULD NEVER BE SEEN");
	}

}
