package org.roboclub.robobuggy.measurments;

import org.opencv.core.Scalar;
import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.numbers.Number;

/**
 * TODO document
 * @author Trevor Decker
 *
 */
public abstract class Measurement<NTYPE extends Number> implements Number{
	NTYPE value;
	unit unit;
	
	@Override
	/**
	 * TODO document
	 */
	public Number sqrt() throws CloneNotSupportedException {
		Measurement<NTYPE> result = (Measurement<NTYPE>) this.clone();
		result.value = (NTYPE) this.value.sqrt();
		return result;
	}

	@Override
	/**
	 * Evaluates to the numbers representation of +1 if the measurement is positive,
	 * Evaluates to the numbers representation of the inverse of 1 if the measurement is negative 
	 */
	public Measurement<NTYPE> signum() throws CloneNotSupportedException {
		Measurement<NTYPE> result = (Measurement<NTYPE>) this.clone();
		result.value = (NTYPE) this.value.signum();
		return result;
	}
	
	@Override
	/**
	 * evaluates to a human readable string encoding this measurement 
	 */
	public String toString(){
		return value.toString()+" "+unit.toString();
	}
	
	@Override
	/**
	 * evaluates to a new measurement of the same type and the same units whose value is the 
	 * calling objects value mod (remainder after division) with someNumber 
	 */
	public Number mod(Number someNumber) throws LogicException, CloneNotSupportedException {
		Measurement<NTYPE> result = (Measurement<NTYPE>) this.clone();
		result.value = (NTYPE) this.value.mod(someNumber);
		return result;
	}
	
	/**
	 * TODO document
	 * @throws LogicException 
	 */
	public Number sub(Number otherNumber) throws LogicException {
		throw new LogicException("it does not make sense to do arithmatic on a distince and a number",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * TODO document 
	 * @throws LogicException 
	 */
	public Number add(Number otherNumber) throws LogicException {
		throw new LogicException("it does not make sense to do arithmatic on a distince and a number",MESSAGE_LEVEL.exception);
	}
	
	@Override
	/**
	 * TODO document
	 */
	public Number inverse() throws CloneNotSupportedException {
		Measurement<NTYPE> result = (Measurement<NTYPE>) this.clone();
		result.value = (NTYPE) this.value.inverse();
		return result;
	}
	
	/***
	 * returns the Units that this measurement is currently using 
	 * @return
	 */
	public unit getUnits(){
		return this.unit;
	}
	
	/***
	 * Overrides the current value of this units units. Note that the measurement value stays the same.
	 * Use setValue to change the measurement. 
	 * @param newUnits
	 */
	public void setUNITS(unit newUnits){
		this.unit = newUnits;
	}
	
	/**
	 * TODO document
	 * @param newValue
	 */
	public void setMeassurmentValue(NTYPE newValue){
		this.value =  newValue;
	}

	/***
	 * returns the scaler(double) numerical value that currently represents the measurement 
	 * @return
	 */
	public Scalar getMeassurmentValue(){
		return (Scalar) this.value;
	}
	
	/**
	 * TODO document
	 * TODO implement
	 * @param s
	 * @return
	 */
	public Measurement<Number> mult(Scalar s){
		return null;
	}
	
	/**
	 * TODO document
	 * TODO implement
	 */
//	public static Number zero() {
//		return null;
//	}
}
