package com.roboclub.robobuggy.nodes.localizers;

import Jama.Matrix;

import com.roboclub.robobuggy.main.Util;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.IMUAngularPositionMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.ui.LocTuple;

import java.util.Date;

/**
 * localizes using a kalman filter
 */
public class KfLocalizer extends PeriodicNode{
	private Matrix motionModel;
	private Publisher posePub;
	private Matrix state;
	private double lastEncoderReading;
//	private Date startTime;
	private Date mostRecentUpdateTime;
	private Matrix covariance;
	private double buggyFrameGpsX;
	private double buggyFrameGpsY;
//	private Matrix matrixDF;
	
	public KfLocalizer(int period) {
		super(new BuggyBaseNode(NodeChannel.POSE), period, "KF");
		posePub = new Publisher(NodeChannel.POSE.getMsgPath());
//		startTime = new Date();
		double[][] startCovariance = {{1000,0,0},{0,1000,0},{0,0,1000}};
		covariance = new Matrix(startCovariance);
		double [][] start = {{-79.9437947},{40.4416841},{-110}};
		state = new Matrix(start);

//		mostRecentUpdateTime = startTime;
		//matrixDF = //TODO
		
        //Initialize subscriber to GPS measurements
        new Subscriber("htGpsLoc", NodeChannel.GPS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                GpsMeasurement newGPSData = (GpsMeasurement)m;
                
              double oldGPSX = buggyFrameGpsX;
              double oldGPSY = buggyFrameGpsY;
             	buggyFrameGpsX = newGPSData.getLongitude();
             	buggyFrameGpsY = newGPSData.getLatitude();
             double dLat = buggyFrameGpsY - oldGPSY;
             double dLon = buggyFrameGpsX - oldGPSX;
             
             double buggyFrameRotZ = Math.toDegrees(Math.atan2(LocalizerUtil.convertLatToMeters(dLat), LocalizerUtil.convertLonToMeters(dLon)));                
             double[][] observationModel = {{5,0,0},{0,5,0},{0,0,5}};
             double[][] meassurement = {{newGPSData.getLongitude()},{newGPSData.getLatitude()},{buggyFrameRotZ}};
             double[][] updateCovariance = {{.5,0,0},{0,.5,0},{0,0,1}};
             updateStep(new Matrix(observationModel),new Matrix(meassurement),new Matrix(updateCovariance));  
                
            }});
        
        
        new Subscriber("HighTrustGpsLoc",NodeChannel.IMU_ANG_POS.getMsgPath(), ((topicName, m) -> {
        	
            IMUAngularPositionMessage mes = ((IMUAngularPositionMessage) m);
            //double y = mes.getRot()[0][1];
            //double x = mes.getRot()[0][0];
            double[][] xVar = {{1},{0},{0}};
            double[][] yVar = {{0},{1},{0}};
            Matrix xMat = new Matrix(xVar);
            Matrix yMat = new Matrix(yVar);
            Matrix rot = new Matrix(mes.getRot());
            double x = rot.times(xMat).get(0, 0);
            double y = rot.times(yMat).get(0, 0);
            double th = -(Math.toDegrees(Math.atan2(y, x))-90);
            th = Util.normalizeAngleDeg((Util.normalizeAngleDeg(th)+state.get(2, 0))/2);
            
       
            
            double[][] observationModel= {{0,0,0},{0,0,0},{0,0,1}};
            double[][] meassurement = {{0},{0},{th}};
            double[][] updateCovariance ={{10000,0,0},{0,10000,0},{0,0,100}};
            
            
  //          updateStep(new Matrix(observationModel),new Matrix(meassurement),new Matrix(updateCovariance));  

        }));
       
        
        // TODO note that we will probably run into precision errors since the changes are so small
        // would be good to batch up the encoder updates until we get a margin that we know can be represented proeprly
        new Subscriber("htGpsLoc", NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                EncoderMeasurement measurement = (EncoderMeasurement) m;
                
                // convert the feet from the last message into a delta degree, and update our position
                double currentEncoderMeasurement = measurement.getDistance();
                double deltaDistance = currentEncoderMeasurement - lastEncoderReading;

                LocTuple deltaPos = LocalizerUtil.convertMetersToLatLng(deltaDistance, state.get(2, 0));
                double X = state.get(0, 0)+deltaPos.getLongitude();
                double Y = state.get(1, 0)+deltaPos.getLatitude();
                double[][] observationMatrix = {{1,0,0},{0,1,0},{0,0,0}};
                double[][] messure = {{X},{Y},{state.get(2, 0)}};
                double[][] cov = {{100,0,0},{0,100,0},{0,0,100}};
                
          //     updateStep(new Matrix(observationMatrix), new Matrix(messure), new Matrix(cov));
                lastEncoderReading = currentEncoderMeasurement;

            }});
        
        new Subscriber("htGpsLoc", NodeChannel.STEERING.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				SteeringMeasurement steerM = (SteeringMeasurement)m;
				// TODO Auto-generated method stub
	             double buggyHeading  = steerM.getAngle();
	             
	                double[][] observationMatrix = {{0,0,0},{0,0,0},{0,0,1}};
	                double[][] messure = {{0},{0},{state.get(2, 0)+buggyHeading}};
	                double[][] cov = {{100,0,0},{0,100,0},{0,0,100}};
//	                updateStep(new Matrix(observationMatrix), new Matrix(messure), new Matrix(cov));

			}
		});
        
        resume();
        
	}
	
	private synchronized void updateStep(Matrix observationMatrix,Matrix measurement,Matrix updateCovariance){
		 	predictStep();

			 Matrix y = measurement.minus(observationMatrix.times(state));
			 Matrix s = observationMatrix.times(covariance).times(observationMatrix.transpose()).plus(updateCovariance);
			 
	
			 Matrix k = covariance.times(observationMatrix.transpose()).times(s.inverse());
			 state = state.plus(k.times(y));
			 covariance = (Matrix.identity(covariance.getRowDimension(), covariance.getColumnDimension()).minus(k.times(observationMatrix)));
		
		
	}

	@Override
	protected void update() {
		 predictStep();
		 System.out.println("publishing"+state.get(0, 0) + ","+state.get(1, 0)+","+state.get(2, 0));
		 //publish state 
		 posePub.publish(new GPSPoseMessage(new Date(), state.get(1, 0), state.get(0,0), state.get(2, 0)));
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
	private  synchronized void predictStep(){
		/*
			Date now = new Date();
			Date diff = new Date(now.getTime() - mostRecentUpdateTime.getTime());
			double dt = (diff.getTime())/1000;
			mostRecentUpdateTime = now;
//			double[][] mModelArray = {{Math.cos(state.get(0, 2))*dt},{Math.sin(state.get(0,2))*dt},{1}};
				motionModel = new Matrix(mModelArray);
			//estimate state
			state =  MotionModel.times(dt).times(state);
			//estimate covariance
			covariance = matrixDF.times(covariance).times(matrixDF.transpose());
			*/
		
		//we don't track velocity so we won't use this 
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

