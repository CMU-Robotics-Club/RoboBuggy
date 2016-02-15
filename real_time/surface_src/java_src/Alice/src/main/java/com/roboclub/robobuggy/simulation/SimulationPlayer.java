package com.roboclub.robobuggy.simulation;

import java.beans.Encoder;
import java.util.Date;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;

//import javafx.scene.control.ButtonBar.ButtonData;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

public class SimulationPlayer extends PeriodicNode{

	
	public SimulationPlayer() {
		super(new BuggyBaseNode(NodeChannel.SIMULATION), 1000);  //TODO figure out why this can't run faster
	//	posePub = new Publisher(NodeChannel.POSE.getMsgPath());
		 Publisher posePub;
		 Publisher encoderPub = new  Publisher(NodeChannel.ENCODER.getMsgPath());
		 Publisher steeringPub = new Publisher(NodeChannel.STEERING.getMsgPath());
		 Publisher imuPub = new Publisher(NodeChannel.IMU.getMsgPath());
		  Thread thread = new Thread(){
			    public void run(){
			    	try {
						this.sleep(5000);
					} catch (InterruptedException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
			    	while(true){
			for(int i = 0;i<1000;i++){
				steeringPub.publish(new  SteeringMeasurement(i));
				encoderPub.publish(new  EncoderMeasurement(i/100.0, 0.0));
				imuPub.publish(new ImuMeasurement(i, i, i));
			    		try {
			    			this.sleep(100);
			    		} catch (InterruptedException e) {
			    			// TODO Auto-generated catch block
			    			e.printStackTrace();
			    		}
			}
			for(int i = 0;i<90;i++){
				steeringPub.publish(new  SteeringMeasurement(90));
				encoderPub.publish(new  EncoderMeasurement(i, 0.0));
				imuPub.publish(new ImuMeasurement(i, i, i));
			    		try {
			    			this.sleep(100);
			    		} catch (InterruptedException e) {
			    			// TODO Auto-generated catch block
			    			e.printStackTrace();
			    		}
			}
			
			    	}
					}
			  };
			  
			  thread.start();
	}
	

int i = 0;
	@Override
	protected void update() {
	//	posePub.publish(new GPSPoseMessage(new Date(), i/100.0, i/50.0, i/45.0));
/*		if(encoderPub != null){
			encoderPub.publish(new  EncoderMeasurement(i/100.0, 0.0));
			i = i+1;
		}
		*/
	}


	@Override
	protected boolean startDecoratorNode() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	protected boolean shutdownDecoratorNode() {
		// TODO Auto-generated method stub
		return false;
	}

}
