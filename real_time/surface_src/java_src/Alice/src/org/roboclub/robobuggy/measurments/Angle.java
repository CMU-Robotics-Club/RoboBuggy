package org.roboclub.robobuggy.measurments;

import java.io.ObjectInputStream.GetField;
import java.lang.reflect.InvocationTargetException;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.numbers.Double_Number;
import org.roboclub.robobuggy.numbers.Number;

/**
 * @author Trevor Decker
 * TODO document
 * @version 0.0
 * 
 */
public class Angle<NTYPE extends  Number> extends Measurement{
	ANGULAR_UNITS unit;
	


	/***
	 * Creates an object that represents a measurement and the units used to make that measurement  
	 * @param units
	 * @param value
	 */
	public Angle(ANGULAR_UNITS units,NTYPE value){
		setUNITS(units);
		setMeassurmentValue(value);
	}

	/***
	 * produces a new unit object which represents the same angle but in degrees
	 * if the calling object is not an angle then an error is thrown 
	 * @return
	 * @throws LogicException 
	 */
	public Angle toDegrees() throws LogicException{
		Angle result = new Angle(this.unit, this.value);
		switch((ANGULAR_UNITS)getUnits()){
		case DEGREES:
			//already in degree so do nothing 
			break;
		case RADIANS:
			result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.2957795)));
			break;
		default:
			throw new LogicException("unit type is not a angle",MESSAGE_LEVEL.exception);
			
		}
		result.setUNITS(ANGULAR_UNITS.DEGREES);
		return result;
	}

	/***
	 * produces a new unit object which represents the same angle but in degrees
	 * if the calling object is not an angle then an error is thrown  
	 * @return
	 * @throws LogicException 
	 */
	public Angle toRadians() throws LogicException{
		Angle result = toDegrees();
		result.setMeassurmentValue(((Number) getMeassurmentValue()).mult(new Double_Number(0.0174532925)));
		return null;
	}

	/**
	 * TODO document 
	 * @throws LogicException 
	 */
	public Angle add(Number otherNumber) throws LogicException {
		throw new LogicException("it does not make sense to add a angle and a number, convert to angle first",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * TODO document 
	 * note result will be an angle in radians
	 * @throws LogicException 
	 */
	public Angle add(Angle otherAngle) throws LogicException {
	    Angle thisAngle = toRadians();
	    otherAngle = otherAngle.toRadians();
	    Number newValue = thisAngle.value.add(otherAngle.value);
		return new Angle(thisAngle.unit, newValue);
	}

	
	/**
	 * TODO document
	 * @throws LogicException 
	 */
	public Angle sub(Number otherNumber) throws LogicException {
		throw new LogicException("it does not make sense to subtract a angle and a number, convert to angle first",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * TODO document
	 * note result will be an angle in radians
	 * @throws LogicException 
	 */
	public Angle sub(Angle otherAngle) throws LogicException {
	    Angle thisAngle = toRadians();
	    otherAngle = otherAngle.toRadians();
	    Number newValue = thisAngle.value.sub(otherAngle.value);
		return new Angle(thisAngle.unit, newValue);
	}

	
	/**
	 * TODO document 
	 * @throws LogicException 
	 */
	public Angle mult(Number otherNumber) throws LogicException {
		return new Angle(this.unit, this.value.mult(otherNumber));
	}

	/**
	 * It does not make sense to multiply an angle by another angle, for now need to create a angle^2 class
	 * @throws LogicException 
	 */
	public Angle mult(Angle otherNumber) throws LogicException {
		throw new LogicException("It does not make sense to multiply an angle by another angle, for now need to create a angle^2 class",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * TODO document
	 */
	public Angle div(Number otherNumber) {
		return new Angle(this.unit, value.div(otherNumber));
	}
	
	/**
	 * TODO document
	 * @throws LogicException 
	 */
	public Number div(Angle otherAngle) throws LogicException {
		Angle thisAngle = toRadians();
		otherAngle = otherAngle.toRadians();
		return thisAngle.value.div(otherAngle.value);
	}

	

	/**
	 * TODO document
	 * @return
	 * @throws LogicException 
	 */
	public static <T extends Number> Angle zero() throws LogicException{
		Double_Number z = Double_Number.zero();
		return new Angle(ANGULAR_UNITS.RADIANS,z);		
		//this is a hack, in future versions any type of number should be usable as the type T
	}

	
	/**
	 * Evaluates to 1 radian, Be careful this is only the true one object 
	 * if the other object is a radian 
	 * @return A Angle object encoding 1 radian
	 */
	public Angle One() {
		Double_Number z = Double_Number.one();
		return new Angle(ANGULAR_UNITS.RADIANS,z);		
		//this is a hack, in future versions any type of number should be usable as the type T
	}

	@Override
	/**
	 * TODO document
	 */
	public boolean isLess(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Angle.class){
			Angle otherAngle = (Angle)otherNumber;
			return value.isLess(otherAngle.value);
		}
		throw new LogicException("can not compare the size of a angle and non angle number",MESSAGE_LEVEL.exception);
	}


	@Override
	/**
	 * TODO document
	 */
	public boolean isGreater(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Angle.class){
			Angle otherAngle = (Angle)otherNumber;
			return value.isGreater(otherAngle.value);
		}
		throw new LogicException("can not compare the size of a angle and non angle number",MESSAGE_LEVEL.exception);

	}


	@Override
	/**
	 * TODO document
	 * TODO implement
	 */
	public boolean isEqual(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Angle.class){
			Angle otherAngle = (Angle)otherNumber;
			return value.isEqual(otherAngle.value);
		}
		throw new LogicException("can not compare the size of a angle and non angle number",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * TODO document 
	 */
	public Angle<Number> clone(){
		return new Angle<Number>(unit, value);
	}

	/** 
	 * Evaluates to a zero angle, will be measured radians 
	 */
	 public Number getZero() throws LogicException{
		 return zero();
	 }

		@Override
		/**
		 * Overides the classes default equal function to fit the number equal function 
		 */
		public boolean equals(Object obj) {
			//try catch is needed since equals can not throw an error 
			try {
				return isEqual((Number) obj);
			} catch (LogicException e) {
				e.printStackTrace();
			}
			return false;
		};

}
