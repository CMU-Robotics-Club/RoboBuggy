package com.roboclub.robobuggy.localization;
import org.opencv.core.Mat;

public class KalmanFilter {
	//TODO implement
	
	private State thisState;
	private Covariance thisCovariance;
	
	public KalmanFilter(){
		reset();
	}
		
	//TODO link in JBlas for matrix operations 
	public void updateStep(Covariance thisCovarance,Mat updateMatrix,Mat measerment){
		//TODO 
	}
	
	public void predictStep(Covariance thisCoavarance,Mat motionModel){
		//TODO 
		
	}
	
	public void reset(){
		thisState = new State();
		thisCovariance = new Covariance();
	}
	
	public Covariance getCovariance(){
		return thisCovariance;
	}
	
	public State getState(){
		return thisState;
	}
	
	
	
}
