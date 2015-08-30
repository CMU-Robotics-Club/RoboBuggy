package org.roboclub.robobuggy.localization;

import java.sql.Time;

import org.roboclub.robobuggy.linearAlgebra.Matrix;
import org.roboclub.robobuggy.linearAlgebra.Vector;

/**
 * TODO document
 * @author Trevor Decker
 * @version 0.0
 *
 */
public class KalmanFilter implements LocalizationFilter{
	private Vector state;
	private Matrix covariance;
	
	/**
	 * TODO document 
	 * TODO implement
	 */
    public KalmanFilter() {
        // This is a very trusting kalman filter; it just adds the
        // encoder offset to the current believed position.
        
    } 
    
    /**
     * TODO document
     * TODO implement
     * @param TransitionModel
     * @param dt
     */
    public void ApplyTransitionModel(Matrix TransitionModel,double dt){
        //TODO
    }
    
    /**
     * TODO document
     * TODO implement
     * @param observationModel
     * @param observation
     */
    public void ApplyObservationModel(Matrix observationModel, Vector observation){
        //TDOO
    }
    
    /**
     * resets the state to newState and confidence newCovariance
     * TODO implement 
     * @param newState
     * @param newCovariance
     */
    public void reset(Vector newState, Matrix newCovariance) {
        // TODO
    }

     /**
      *  evaluates to the current state of the klamanFilter
      *  if you want the most up to date value run applyTransionModel to the current time
      * @return
      */
    public Vector getState(){
        return state;
    }

     /**
      *  evaluates to the current confidence of the kalmanFilter
      *  if you want the most up to date value run applyTransitionModel to the current time
      * @return
      */
    public Matrix getConfidence(){
        return covariance;
    }

	@Override
	/**
	 * TODO implement
	 * TODO document
	 */
	public void addSensorMeaserment(SensorMeasurment aMeasurment) {
		// TODO Auto-generated method stub
		
	}

	@Override
	/**
	 * TODO implement
	 * TODO document
	 */
	public LocationEstimate getEstimateAt(Time atime) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/**
	 * TODO implement
	 * TODO document 
	 */
	public void reInitSystem(LocationEstimate initState) {
		// TODO Auto-generated method stub
		
	}
}
