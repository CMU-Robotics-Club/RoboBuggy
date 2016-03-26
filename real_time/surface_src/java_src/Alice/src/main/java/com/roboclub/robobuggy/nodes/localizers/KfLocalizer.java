package com.roboclub.robobuggy.nodes.localizers;

import Jama.Matrix;
import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import java.util.Date;

/**
 * localizes using a kalman filter
 */
public class KfLocalizer extends PeriodicNode{
//	private MotionModel mModel;
	private Publisher posePub;
	private Matrix state;
//	private Date startTime;
//	private Date mostRecentUpdateTime;
	private Matrix covariance;
//	private Matrix matrixDF;
	
	protected KfLocalizer(BuggyNode base, int period) {
		super(base, period);
		posePub = new Publisher(NodeChannel.POSE.getMsgPath());
//		startTime = new Date();
		double[][] startCovariance = {{1,0,0},{0,1,0},{0,0,1}};
		covariance = new Matrix(startCovariance);
		state = Util.createIdentityMatrix(3);
//		mostRecentUpdateTime = startTime;
		//matrixDF = //TODO
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

	/**
	 * TODO
	 */
	//TODO
	private void predictStep(){
		
//			Date now = new Date();
//			Date dt = new Date(now.getTime() - mostRecentUpdateTime.getTime());
			//estimate state
//			state =  mModel.applyMotionModel(state,dt);
			//estimate covariance
//			covariance = matrixDF.times(covariance).times(matrixDF.transpose());
		}


	/**
	 * TODO
	 * @param measurement TODO
	 * @param oModel TODO
	 * @param matrixDH TODO
	 */
	//TODO
	public void updateStep(Matrix measurement,ObservationModel oModel,Matrix matrixDH){
		//run predict to get to the current time 
		predictStep();  //TODO consider sending time to predict to 
		
		
		Matrix inovation = measurement.minus(oModel.getObservationSpaceState(state));
		Matrix innovationCovariance = matrixDH.times(covariance).times(matrixDH.transpose());
		Matrix kalmanGain = covariance.times(matrixDH.transpose()).times(innovationCovariance.inverse());
		covariance = (Util.createIdentityMatrix(3).minus(kalmanGain.times(matrixDH)).times(covariance));
		state = state.plus(kalmanGain.times(inovation));
//		mostRecentUpdateTime = new Date();
	}


}

