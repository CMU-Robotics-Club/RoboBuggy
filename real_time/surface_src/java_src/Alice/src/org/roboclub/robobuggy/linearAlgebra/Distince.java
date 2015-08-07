package org.roboclub.robobuggy.linearAlgebra;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;

/**
 * 
 * @author Trevor Decker
 * @version 0.0
 * object that represents a distance, encodes both a scalar along an axis of 
 * and the units of that measurement, can convert between different units and
 *  do math operations which will automatically convert units for you. 
 */
public class Distince implements Number{
	Number value;
	DISTINCE_UNITS unit;


	/***
	 * Creates an object that represents a measurement and the units used to make that measurement  
	 * @param units
	 * @param value
	 */
	public Distince(DISTINCE_UNITS units,Number value){
		setUNITS(units);
		setMeassurmentValue(value);
	}
	
	public void setMeassurmentValue(Number newValue){
		this.value = newValue;
	}


	/***
	 * returns the scaler(double) numerical value that currently represents the measurement 
	 * @return
	 */
	public Number getMeassurmentValue(){
		return this.value;
	}

	/***
	 * Overrides the current value of this units units. Note that the measurement value stays the same.
	 * Use setValue to change the measurement. 
	 * @param newUnits
	 */
	public void setUNITS(DISTINCE_UNITS newUnits){
		this.unit = newUnits;
	}
	
	
	/***
	 * produces a new unit object which represents the same distance but in millimeters
	 * if the calling object is not a distance then an error is thrown 
	 * @return
	 * @throws LogicException 
	 */
	public Distince toMillimeters() throws LogicException{
		Distince result = this.toMeters();
		result.setMeassurmentValue(new Double_Number(1000).mult(getMeassurmentValue()));
		result.setUNITS(DISTINCE_UNITS.MILLIMETERS);
		return result;
	}

	/***
	 * produces a new unit object which represents the same distance but in centimeters
	 * if the calling object is not a distance then an error is thrown 
	 * @return
	 * @throws LogicException 
	 */
	public Distince toCentimeters() throws LogicException{
		Distince result = this.toMeters();
		result.setMeassurmentValue(new Double_Number(100).mult(getMeassurmentValue()));
		result.setUNITS(DISTINCE_UNITS.CENTIMETERS);
		return result;	
	}

	/***
	 * produces a new unit object which represents the same distance but in meters
	 * if the calling object is not a distance then an error is thrown 
	 * @return
	 * @throws LogicException 
	 */
	public Distince toMeters() throws LogicException{
		Distince result = new Distince(this.unit, this.value);
		switch(getUnits()){
			case CENTIMETERS:
				result.setMeassurmentValue(new Double_Number(0.01).mult(getMeassurmentValue()));
				break;
			case MILLIMETERS :
				result.setMeassurmentValue(new Double_Number(0.001).mult(getMeassurmentValue()));
				break;
			case METERS:
				//DO noting becasue the units are already meters	
				break;
			case KILIOMETERS:
				result.setMeassurmentValue(new Double_Number(1000).mult(getMeassurmentValue()));
				break;
			case INCHES:
				result.setMeassurmentValue(new Double_Number(0.0254).mult(getMeassurmentValue()));
				break;
			case FEET:
				result.setMeassurmentValue(new Double_Number(0.3048).mult(getMeassurmentValue()));
				break;
			case MILES:
				result.setMeassurmentValue(new Double_Number(1609.34).mult(getMeassurmentValue()));
				break;
			default:
				throw new LogicException("unit type is not a distince",MESSAGE_LEVEL.exception);
		}
		result.setUNITS(DISTINCE_UNITS.METERS);
		return result;
	}

	/***
	 * produces a new unit object which represents the same distance but in kilometers
	 * if the calling object is not a distance then an error is thrown 
	 * @return
	 * @throws LogicException 
	 */
	public Distince toKilometers() throws LogicException{
		Distince result = this.toMeters();
		result.setMeassurmentValue(new Double_Number(0.001).mult(getMeassurmentValue()));
		result.setUNITS(DISTINCE_UNITS.KILIOMETERS);
		return result;
	}

	/***
	 * produces a new unit object which represents the same distance but in Inches
	 * if the calling object is not a distance then an error is thrown 
	 * @return
	 * @throws LogicException 
	 */
	public Distince toInches() throws LogicException{
		Distince result = this.toMeters();
		result.setMeassurmentValue(new Double_Number(0.3700787).mult(getMeassurmentValue()));
		result.setUNITS(DISTINCE_UNITS.INCHES);
		return result;
	}

	/***
	 * produces a new unit object which represents the same distance but in feet
	 * if the calling object is not a distance then an error is thrown 
	 * @return
	 * @throws LogicException 
	 */
	public Distince toFeet() throws LogicException{
		Distince result = this.toMeters();
		result.setMeassurmentValue(new Double_Number(0.28084).mult(getMeassurmentValue()));
		result.setUNITS(DISTINCE_UNITS.FEET);
		return result;
	}

	/***
	 * produces a new unit object which represents the same distance but in miles
	 * if the calling object is not a distance then an error is thrown 
	 * @return
	 * @throws LogicException 
	 */
	public Distince toMiles() throws LogicException{
		Distince result = this.toMeters();
		result.setMeassurmentValue(new Double_Number(0.000621371).mult(getMeassurmentValue()));
		result.setUNITS(DISTINCE_UNITS.MILES);
		return result;
	}
	
	/***
	 * returns the Units that this measurement is currently using 
	 * @return
	 */
	public DISTINCE_UNITS getUnits(){
		return this.unit;
	}

	
	/**
	 * TODO document 
	 * @throws LogicException 
	 */
	public Distince add(Number otherNumber) throws LogicException {
		throw new LogicException("it does not make sense to do arithmatic on a distince and a number",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * TODO document
	 * @param otherNumber
	 * @return
	 * @throws LogicException 
	 */
	public Distince add(Distince otherNumber) throws LogicException{
		return new Distince(this.unit, this.value.add(otherNumber.value));
	}

	/**
	 * TODO document
	 * @throws LogicException 
	 */
	public Distince sub(Number otherNumber) throws LogicException {
		throw new LogicException("it does not make sense to do arithmatic on a distince and a number",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * TODO document
	 * @throws LogicException 
	 */
	public Distince sub(Distince otherDistince) throws LogicException{
		return new Distince(this.unit, this.value.sub(otherDistince.value));
	}

	/**
	 * TODO document 
	 * @throws LogicException 
	 */
	public Distince mult(Number otherNumber) throws LogicException {
		return new Distince(this.unit, this.value.mult(otherNumber));

	}
	
	/**
	 * TODO document
	 * @param otherDistince
	 * @return
	 * @throws LogicException 
	 */
	public Distince mult(Distince otherDistince) throws LogicException{
		throw new LogicException("it does not make sense multipy a distince by a distince",MESSAGE_LEVEL.exception);
	}


	/**
	 * TODO document
	 */
	public Distince div(Number otherNumber) {
		return new Distince(this.unit, this.value.div(otherNumber));
	}
	
	/**
	 * TODO document
	 */
	public Number div(Distince otherDistince){
		return this.value.div(otherDistince.value);
	}

	/**
	 * TODO document
	 */
	public static Distince zero() {
		//TODO come up with a better solution so that other numebr types can be used 
		Number z = Double_Number.zero();
		return new Distince(DISTINCE_UNITS.METERS, z);
	}

	/**
	 * TODO document 
	 */
	public static Distince one() {
		//TODO come up with a better solution so that other numebr types can be used 
		Number one = Double_Number.one();
		return new Distince(DISTINCE_UNITS.METERS, one);
	}
	
	@Override
	/**
	 * TODO document
	 */
	public Distince inverse() {
		return new Distince(this.unit, this.value.inverse());
	}
	
	@Override
	/**
	 * TODO implement
	 * TODO document 
	 */
	public Distince mod(Number someNumber) throws LogicException {
		return new Distince(this.unit, this.value.mod(someNumber));
	}
	
	@Override
	/**
	 * TODO documnet
	 */
	public boolean isLess(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Distince.class){
			Distince otherDistince = (Distince)otherNumber;
			return value.isLess(otherDistince.value);
		}
		throw new LogicException("can not compare the size of a distince and non distince number",MESSAGE_LEVEL.exception);
	}
	
	@Override
	/**
	 * TODO document 
	 */
	public boolean isGreater(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Distince.class){
			Distince otherDistince = (Distince)otherNumber;
			return value.isGreater(otherDistince.value);
		}
		throw new LogicException("can not compare the size of a distince and non distince number",MESSAGE_LEVEL.exception);
	}
	
	@Override
	/**
	 * TODO document
	 */
	public boolean isEqual(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Distince.class){
			Distince otherDistince = (Distince)otherNumber;
			return value.isEqual(otherDistince.value);
		}
		throw new LogicException("can not compare the size of a distince and non distince number",MESSAGE_LEVEL.exception);
	}
	
	@Override
	/**
	 * TODO document
	 */
	public Distince sqrt() {
		return new Distince(this.unit, value.sqrt());
	}
	
	@Override
	/**
	 * Evaluates to the numbers representation of +1 if the Distance is positive,
	 *  Evaluates to the numbers representation of the inverse of 1 if the number is negative 
	 */
	public Distince signum() throws CloneNotSupportedException {
		return new Distince(this.unit, value.signum());
	}

}
