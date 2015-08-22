package org.roboclub.robobuggy.measurments;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.numbers.Double_Number;
import org.roboclub.robobuggy.numbers.Number;
import org.roboclub.robobuggy.numbers.Scalar;

/**
 * 
 * @author Trevor Decker
 * @version 0.0
 * object that represents a distance, encodes both a scalar along an axis of 
 * and the units of that measurement, can convert between different units and
 *  do math operations which will automatically convert units for you. 
 */
public class Distince extends  Measurement<Number>{
	Number value;


	/***
	 * Creates an object that represents a measurement and the units used to make that measurement  
	 * @param units
	 * @param value
	 */
	public Distince(unit units,Number value){
		setUNITS((DISTINCE_UNITS) units);
		setMeassurmentValue(value);
	}


	
	
	/***
	 * produces a new unit object which represents the same distance but in millimeters
	 * if the calling object is not a distance then an error is thrown 
	 * @return
	 * @throws LogicException 
	 */
	public Distince toMillimeters() throws LogicException{
		Distince result = this.toMeters();
		result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(1000)));
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
		result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(100)));
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
		switch((DISTINCE_UNITS)getUnits()){
			case CENTIMETERS:
				result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.01)));
				break;
			case MILLIMETERS :
				result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.001)));
				break;
			case METERS:
				//DO noting becasue the units are already meters	
				break;
			case KILIOMETERS:
				result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(1000)));
				break;
			case INCHES:
				result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.0254)));
				break;
			case FEET:
				result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.3048)));
				break;
			case MILES:
				result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(1609.34)));
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
		result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.001)));
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
		result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.3700787)));
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
		result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.28084)));
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
		result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.000621371)));
		result.setUNITS(DISTINCE_UNITS.MILES);
		return result;
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
	 * Todo implement
	 * todo document
	 * @param dt
	 * @return
	 */
	public DistincePerTime div(Time dt){
		return null;
	}
	
	/**
	 * TODO document
	 */
	public Number div(Distince otherDistince){
		return this.value.div(otherDistince.value);
	}
	
	/**
	 * TODO implement
	 * TODO document
	 * @param num
	 * @return
	 */
	public Distince div(Scalar num){
		return null;
	}

	/**
	 * evaluates to an object representing zero distance
	 */
	public static Distince zero() {
		//TODO come up with a better solution so that other number types can be used 
		Number z = Double_Number.zero();
		return new Distince(DISTINCE_UNITS.METERS, z);
	}

	/**
	 * evaluates to an object representing 1 distance using the portamento distance unit
	 */
	public static Distince one() {
		//TODO come up with a better solution so that other number types can be used 
		Number one = Double_Number.one();
		return new Distince(DISTINCE_UNITS.METERS, one);
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
	 * Overrides the classes default equal function to fit the number equal function 
	 */
	public boolean equals(Object obj) {
		try {
			return isEqual((Number) obj);
		} catch (LogicException e) {
			e.printStackTrace();
		}
		return false;
	};

}
