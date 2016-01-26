package com.roboclub.robobuggy.localization;

/**
 * Class used to represent a Kalman Filter
 */
public class KalmanFilter {
//	private vector state;
//	private matrix covariance;
	
	/**
	 * Construct a new instance of a {@link KalmanFilter}
	 */
    public KalmanFilter() {
        // This is a very trusting kalman filter; it just adds the
        // encoder offset to the current believed position.
        
    }
  /*  
    public void ApplyTransitionModel(matrix TransitionModel,double dt){
        //TODO
    }
    
    public void ApplyObservationModel(matrix observationModel, vector observation){
        //TDOO
    }
    
    /// resets the state to newState and confidence newCovariance
    public void reset(vector newState, matrix newCovariance) {
        // TODO
    }

     // evaluates to the current state of the klamanFilter
     // if you want the most up to date value run applyTransionModel to the current time
    public vector getState(){
        return state;
    }

     // evaluates to the current confidence of the kalmanFilter
     // if you want the most up to date value run applyTransitionModel to the current time
    public matrix getConfidence(){
        return covariance;
    }
    */
}
