package org.roboclub.robobuggy.coordinateFrame2;

/**
 * TODO document
 * @author Trevor Decker
 *
 */
public class StateVector{
	double[] vector;
	States[] states;
	
	/**
	 * TODO document 
	 * @param newVector
	 * @param newstates
	 */
	public StateVector(double[] newVector,States[] newstates){
		vector = newVector;
		states = newstates;
	}
	
	/**
	 * TODO document 
	 * TODO implement 
	 * @param stateToGet
	 * @return
	 */
	public double getState(States stateToGet){
		return 0;
		//TODO
	}
	
	/**
	 * TODO document 
	 * TOOD implement 
	 * @param stateToSet
	 * @param newValue
	 */
	public void setState(States stateToSet,double newValue){
		//TODO
	}
	
	
	/**
	 * evaluates to a number representing the distance between two state vectors  
	 * TOOD implement
	 * @return
	 */
	public double getDistince(StateVector otherStateVector){
		//TODO
		return 0;
	}
}
