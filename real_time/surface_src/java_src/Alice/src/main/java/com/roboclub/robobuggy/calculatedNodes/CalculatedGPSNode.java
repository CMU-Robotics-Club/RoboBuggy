package com.roboclub.robobuggy.calculatedNodes;

import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.sensors.SensorState;

public class CalculatedGPSNode extends BaseCalculatedNode {
	
	private Publisher statePub;
	private Publisher msgPub;
	
	public CalculatedGPSNode (SensorChannel sensor, NodeCalculator calc, int delay) {
		msgPub = new Publisher(sensor.getMsgPath());
		statePub = new Publisher(sensor.getStatePath());
		statePub.publish(new StateMessage(SensorState.DISCONNECTED));
		statePub.publish(new StateMessage(SensorState.ON));
		this.calc = calc;
		this.delay = delay;
		
//		msgPub.publish(new ImuMeasurement(250, 250, 250));
	}
	
	@Override
	public void crunch () {
		new Thread(new Runnable() {
			public void run() {
				while(true) {
					//I'm sensing problems can arise here ... 
					if (enabled) {
						msgPub.publish((GpsMeasurement)calc.calculator(elapsed));
					}
									
					elapsed += delay;
					
					try {
						Thread.sleep(delay);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		}).start();
	}
}
