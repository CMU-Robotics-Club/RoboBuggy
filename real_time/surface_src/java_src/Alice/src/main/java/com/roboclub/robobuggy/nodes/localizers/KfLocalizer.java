package com.roboclub.robobuggy.nodes.localizers;

import java.util.ArrayList;
import java.util.Date;

import Jama.Matrix;

import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

public class KfLocalizer extends PeriodicNode{	
	private MotionModel mModel;
	private Publisher posePub;
	private Matrix state;
	private Date startTime;
	private Date mostRecentUpdateTime;
	private Matrix covariance;
	private Matrix DF;
	
	protected KfLocalizer(BuggyNode base, int period) {
		super(base, period);
		posePub = new Publisher(NodeChannel.POSE.getMsgPath());
		startTime = new Date();
		double[][] startCovariance = {{1,0,0},{0,1,0},{0,0,1}};
		covariance = new Matrix(startCovariance);
		state = Util.eye(3);
		mostRecentUpdateTime = startTime;
		//DF = //TODO
	}

	@Override
	protected void update() {
		 predictStep();
		 //publish state 
		 posePub.publish(new GPSPoseMessage(new Date(), state.get(0, 0), state.get(1,0), state.get(2, 0)));
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
	
	//TODO
	private void predictStep(){
		
			Date now = new Date();
			Date dt = new Date(now.getTime() - mostRecentUpdateTime.getTime());
			
			//estimate state
			state =  mModel.applyMotionModel(state,dt);
			//estimate covariance
			covariance = DF.times(covariance).times(DF.transpose());
		}
	
	
	//TODO
	public void updateStep(Matrix measurement,ObservationModel oModel,Matrix DH){
		//run predict to get to the current time 
		predictStep();  //TODO consider sending time to predict to 
		
		
		Matrix inovation = measurement.minus(oModel.getObservationSpaceState(state));
		Matrix innovationCovariance = DH.times(covariance).times(DH.transpose());
		Matrix kalmanGain = covariance.times(DH.transpose()).times(innovationCovariance.inverse());
		covariance = (Util.eye(3).minus(kalmanGain.times(DH)).times(covariance));
		state = state.plus(kalmanGain.times(inovation));
		mostRecentUpdateTime = new Date();
	}


}

