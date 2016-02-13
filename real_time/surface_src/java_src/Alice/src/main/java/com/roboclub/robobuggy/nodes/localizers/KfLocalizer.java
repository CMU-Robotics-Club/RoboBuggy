package com.roboclub.robobuggy.nodes.localizers;

import java.util.Date;

import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
/*
public class KfLocalizer extends PeriodicNode{
	
	
	private MotionModel mModel;
	private Publisher posePub;
	private State thisState;

	
	
	protected KfLocalizer(BuggyNode base, int period) {
		super(base, period);
		posePub = new Publisher(NodeChannel.POSE.getMsgPath());
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void update() {
		 predictStep();
		 //publish state 
		 posePub.publish(new PoseMessage(new Date(), thisState.getX(), thisState.geyY(), thisState.getHeading()));
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
		//estimate state
		state = mModel.applyMotionModel(state,time);
		//estimate covariance
		covariance = DF*covariance*DF.transpose;
	}
	
	//TODO
	public void updateStep(ObservationModel oModel){
		 predictStep();

		measurement  = ?
		
		inovation = measurement - oModel(state);
		innovationCovariance = DH*covariance*DH.transpose;
		kalmanGain = covariance*DH.transpose*innovationCovariance.inverse;
		covariance = (eye - kalmanGain*DH)*covariance;
		state = state+kalmanGain*inovation;
			
	}
	
	//TODO set motion model                    
	//TODO set observation model


}
*/

