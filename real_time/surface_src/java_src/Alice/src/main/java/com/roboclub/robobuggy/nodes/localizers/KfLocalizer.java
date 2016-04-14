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

import java.util.Date;

/**
 * localizes using a kalman filter
 */
public class KfLocalizer extends PeriodicNode{
	private Publisher posePub;
	private Matrix state;
	private double lastEncoderReading;
	private long lastEncoderReadingTime;
	private Matrix covariance;
	private UTMTuple lastGPS;
	private Matrix motionMatrix;
	private Date mostRecentUpdateTime;
	private Matrix predictCovariance;
	private double wheelBase;
	
	public KfLocalizer(int period) {
		super(new BuggyBaseNode(NodeChannel.POSE), period, "KF");
		posePub = new Publisher(NodeChannel.POSE.getMsgPath());
		wheelBase = 1.13; //meters

		double[][] startCovariance = {
					{1,0,0,0,0,0,0}, //x
					{0,1,0,0,0,0,0}, //y
					{0,0,1,0,0,0,0},    //x_b
					{0,0,0,1,0,0,0},  //y_b
					{0,0,0,0,1,0,0}, //th
					{0,0,0,0,0,1,0},  //th_dot
					{0,0,0,0,0,0,1}   //heading
					};
		covariance = new Matrix(startCovariance);
		//state [x,y,x_b_dot,y_b_dot,th,th_dot,gamma]
		LocTuple startLatLng = new LocTuple(40.4416651, -79.9437577);
		UTMTuple startUTM = LocalizerUtil.Deg2UTM(startLatLng);
		lastGPS = startUTM;
		lastEncoderReadingTime = new Date().getTime();
		mostRecentUpdateTime = new Date();
		
		double [][] start = {{startUTM.Easting},   // X meters
							 {startUTM.Northing},  // Y meters 
							 {0},                  // x_b_dot
							 {0},                  // y_b_dot
							 {-110},		       // th degree
							 {0},			       // th_dot degrees/second
							 {0}          	       // heading degree
		};
		state = new Matrix(start);
		
		double[][] predictCovarianceArray = {
				{1, 0, 0, 0, 0, 0, 0}, //x
				{0, 1, 0, 0, 0, 0, 0}, //y
				{0, 0, 1, 0, 0, 0, 0}, //x_b
				{0, 0, 0, 1, 0, 0, 0}, //y_b
				{0, 0, 0, 0, 1, 0, 0}, //th
				{0, 0, 0, 0, 0, 1, 0}, //th_dot
				{0, 0, 0, 0, 0, 0, 1} //heading 
		};
		predictCovariance= new Matrix(predictCovarianceArray);
		
		
        //Initialize subscriber to GPS measurements
        new Subscriber("htGpsLoc", NodeChannel.GPS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
              GpsMeasurement newGPSData = (GpsMeasurement)m; 
          	  LocTuple gpsLatLng = new LocTuple(newGPSData.getLatitude(),newGPSData.getLongitude());
    		  UTMTuple gpsUTM = LocalizerUtil.Deg2UTM(gpsLatLng);
              double dx = gpsUTM.Easting - lastGPS.Easting;
              double dy = gpsUTM.Northing - lastGPS.Northing;
              double th = Math.toDegrees(Math.atan2(dy, dx));
              lastGPS = gpsUTM;
             
              double[][] observationModel = {{1,0,0,0,0,0,0}, //x
            		                         {0,1,0,0,0,0,0}, //y
            		                         {0,0,0,0,0,0,0}, //x_dot_b
            		                         {0,0,0,0,0,0,0}, //y_dot_b
            		                         {0,0,0,0,1,0,0}, //th
            		                         {0,0,0,0,0,0,0}, //th_dot
            		                         {0,0,0,0,0,0,0} //Heading
            };
              
              //don't update angle if we did not move a lot
              if(Math.sqrt(dx*dx + dy*dy) < .5){ 
            	  observationModel[4][4] = 0;
              }
              
              double[][] meassurement = {{gpsUTM.Easting},
            		                     {gpsUTM.Northing},
            		                     {0},
            		                     {0},
            		                     {th},
            		                     {0},
            		                     {0}
              };
              double[][] updateCovariance = {{1,  0, 0, 0, 0, 0, 0},  //x
            		                         {0,  1, 0, 0, 0, 0, 0},  //y
            		                         {0,   0, 1, 0, 0, 0, 0},  //x_dot
            		                         {0,   0, 0, 1, 0, 0, 0},  //y_dot
            		                         {0,   0, 0, 0, 1, 0, 0},  //th
            		                         {0,   0, 0, 0, 0, 1, 0},  //th_dot  
            		                         {0,   0, 0, 0, 0, 0, 1}   //heading
            		                         };
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
                 double[][] observationModel = {{0,0,0,0,0,0,0}, //x
                        {0,0,0,0,0,0,0}, //y
                        {0,0,0,0,0,0,0}, //x_dot_b
                        {0,0,0,0,0,0,0}, //y_dot_b
                        {0,0,0,0,1,0,0}, //th
                        {0,0,0,0,0,0,0}, //th_dot
                        {0,0,0,0,0,0,0} //Heading
                        };

                double[][] meassurement = {{0},
                    {0},
                    {0},
                    {0},
                    {th},
                    {0},
                    {0}
                };
                double[][] updateCovariance = {
                		{1,  0, 0, 0, 0, 0, 0},  //x
                        {0,  1, 0, 0, 0, 0, 0},  //y
                        {0,   0, 1, 0, 0, 0, 0},  //x_dot
                        {0,   0, 0, 1, 0, 0, 0},  //y_dot
                        {0,   0, 0, 0, 1, 0, 0},  //th
                        {0,   0, 0, 0, 0, 1, 0},  //th_dot  
                        {0,   0, 0, 0, 0, 0, 1}   //heading
                        };
              //  updateStep(new Matrix(observationModel),new Matrix(meassurement),new Matrix(updateCovariance)); 

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
                long currentTime = new Date().getTime();
                long dt = currentTime - lastEncoderReadingTime;
                if(dt > 1){ //to remove numeric instability 
                double bodySpeed = deltaDistance/(dt/1000.0);
                lastEncoderReadingTime = currentTime;
                lastEncoderReading = currentEncoderMeasurement;

                double[][] observationModel = {
                		{0,0,0,0,0,0,0}, //x
                        {0,0,0,0,0,0,0}, //y
                        {0,0,1,0,0,0,0}, //x_dot_b
                        {0,0,0,0,0,0,0}, //y_dot_b
                        {0,0,0,0,0,0,0}, //th
                        {0,0,0,0,0,0,0}, //th_dot
                        {0,0,0,0,0,0,0} //Heading
                        };

                double[][] meassurement = {{0},
                    {0},
                    {bodySpeed},
                    {0},
                    {0},
                    {0},
                    {0}
                };
                double[][] updateCovariance = {
                		{1,  0, 0, 0, 0, 0, 0},  //x
                        {0,  1, 0, 0, 0, 0, 0},  //y
                        {0,   0, 1, 0, 0, 0, 0},  //x_dot
                        {0,   0, 0, 1, 0, 0, 0},  //y_dot
                        {0,   0, 0, 0, 1, 0, 0},  //th
                        {0,   0, 0, 0, 0, 1, 0},  //th_dot  
                        {0,   0, 0, 0, 0, 0, 1}   //heading
                        };
                updateStep(new Matrix(observationModel),new Matrix(meassurement),new Matrix(updateCovariance));  
                }
            }});
        
        new Subscriber("htGpsLoc", NodeChannel.STEERING.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				SteeringMeasurement steerM = (SteeringMeasurement)m;
                double[][] observationModel = {{0,0,0,0,0,0,0}, //x
                        {0,0,0,0,0,0,0}, //y
                        {0,0,0,0,0,0,0}, //x_dot_b
                        {0,0,0,0,0,0,0}, //y_dot_b
                        {0,0,0,0,0,0,0}, //th
                        {0,0,0,0,0,0,0}, //th_dot
                        {0,0,0,0,0,0,1} //Heading
                        };

                double[][] meassurement = {{0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {0},
                    {steerM.getAngle()}
                };
                double[][] updateCovariance = {
                		{1,  0, 0, 0, 0, 0, 0},  //x
                        {0,  1, 0, 0, 0, 0, 0},  //y
                        {0,   0, 1, 0, 0, 0, 0},  //x_dot
                        {0,   0, 0, 1, 0, 0, 0},  //y_dot
                        {0,   0, 0, 0, 1, 0, 0},  //th
                        {0,   0, 0, 0, 0, 1, 0},  //th_dot  
                        {0,   0, 0, 0, 0, 0, 1}   //heading
                        };
                updateStep(new Matrix(observationModel),new Matrix(meassurement),new Matrix(updateCovariance)); 
			}
		});
		
        
        resume();
        
	}
	
	private synchronized void updateStep(Matrix observationMatrix,Matrix measurement,Matrix updateCovariance){
		 	predictStep();
			Matrix inovation = measurement.minus(observationMatrix.times(state));
			Matrix innovationCovariance = observationMatrix.times(covariance).times(observationMatrix.transpose()).plus(updateCovariance);
			Matrix kalmanGain = covariance.times(observationMatrix.transpose()).times(innovationCovariance.inverse());
			state = state.plus(kalmanGain.times(inovation));
			covariance = (Matrix.identity(covariance.getRowDimension(), covariance.getColumnDimension()).minus(kalmanGain.times(observationMatrix)));
	}

	@Override
	protected void update() {
		 predictStep();
		 //publish state 
		 UTMTuple currentLatLng = new UTMTuple(17, 'T', state.get(0, 0), state.get(1, 0));
		 LocTuple latLng = LocalizerUtil.UTM2Deg(currentLatLng);
		 posePub.publish(new GPSPoseMessage(new Date(), latLng.getLatitude(), latLng.getLongitude(), state.get(4, 0)));
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
		Date now = new Date();
		double diff = (now.getTime() - mostRecentUpdateTime.getTime())/1000.0;
		mostRecentUpdateTime = now;
		double th = Math.toRadians(state.get(4, 0));
		double heading = Math.toRadians(state.get(6, 0));
  		double[][] motionModel = {
  			//   x  y x_b y_b th th_dot heading
				{1, 0, Math.cos(th)*diff, - Math.sin(th)*diff, 0, 0, 0}, //x
				{0, 1, Math.sin(th)*diff,    Math.cos(th)*diff, 0, 0, 0}, //y
				{0, 0, 1, 0, 0, 0, 0}, //x_b
				{0, 0, 0, 1, 0, 0, 0}, //y_b
				{0, 0, 0, 0, 1, diff, 0}, //th
				{0, 0, 180/(Math.PI*(wheelBase/Math.sin(heading))), 0, 0, 0, 0}, //th_dot
				{0, 0, 0, 0, 0, 0, 1} //heading
		};
		motionMatrix = new Matrix(motionModel);
		state = motionMatrix.times(state);
		covariance = predictCovariance.times(covariance).times(predictCovariance.transpose());
		}
	

}

