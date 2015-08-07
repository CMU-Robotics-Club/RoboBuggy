package org.roboclub.robobuggy.linearAlgebra;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;

/**
 * 
 * @author Trevor Decker
 * @version 0.0
 * 
 * A Integer which implements the Number class. 
 * This is so that the number can be used with 
 * code designed for the number interface 
 *
 */
public class Integer_Number implements Number{
	private int value;
	/**
	 * TODO document 
	 * @param numberValue
	 */
	public Integer_Number(int numberValue) {
		this.value = numberValue;
	}

	@Override
	/**
	 * Evaluates to a number representation of this number added to the other number, 
	 * order of addition should not matter
	 */
	public Integer_Number add(Number otherNumber) throws LogicException {
		return new Integer_Number(value+(otherNumber.toInteger_Number()).getValue());
	}
	
	/**
	 * TODO document 
	 * @return
	 */
	public int getValue(){
		return value;
	}

	@Override
	/**
	 * TODO document 
	 * @param Number other Number
	 */
	public Integer_Number sub(Number otherNumber) {
		return  new Integer_Number(value-((Integer_Number) otherNumber).getValue());
	}

	@Override
	/**
	 * TODO document
	 * @param Number other Number 
	 */
	public Integer_Number mult(Number otherNumber) {
		return  new Integer_Number(value*((Integer_Number) otherNumber).getValue());
	}

	@Override
	/**
	 * TODO document 
	 * @param Number otherNumber
	 */
	public Integer_Number div(Number otherNumber) {
		return new Integer_Number(value/((Integer_Number) otherNumber).getValue());
	}

	/**
	 * TODO document
	 */
	public static Integer_Number zero() {
		return new Integer_Number(0);
	}
	
	/**
	 * TODO document 
	 */
	public static Integer_Number One() {
		return new Integer_Number(1);
	}

	@Override
	/**
	 * evaluates to the inverse representation of the number,
	 * in this case -1*the calling objects value
	 */
	public Integer_Number inverse() {
		return new Integer_Number(-value);
	}

	@Override
	/**
	 * TODO document
	 * @param Number someNumber
	 */
	public Integer_Number mod(Number someNumber) throws LogicException {
		if(someNumber.getClass() != Integer_Number.class){
			throw new LogicException("trying to mod an integer number by a number type that does not make sense",MESSAGE_LEVEL.exception);
		}
		Integer_Number otherInteger = (Integer_Number)someNumber;
		return new Integer_Number(value % otherInteger.value);
	}

	@Override
	/**
	 * TODO document
	 * @param Number otherNumber
	 */
	public boolean isLess(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Integer_Number.class){
			Integer_Number otherInteger = (Integer_Number)otherNumber;
			return value < otherInteger.value; 
		}else if(this.isLess(otherNumber)){
			return true;
		}
		//otherNumber type does not make sense
		throw new LogicException("trying check equality to an  integer number by a number type that does not make sense",MESSAGE_LEVEL.exception);
	}

	@Override
	/**
	 * TODO document
	 * @param Number otherNumber
	 */
	public boolean isGreater(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Integer_Number.class){
			Integer_Number otherInteger = (Integer_Number)otherNumber;
			return value > otherInteger.value; 
		}else if(this.isGreater(otherNumber)){
			return true;
		}
		//otherNumber type does not make sense
		throw new LogicException("trying check equality to an  integer number by a number type that does not make sense",MESSAGE_LEVEL.exception);

	}

	@Override
	/**
	 * TODO document
	 * @param  Number otherNumber
	 */
	public boolean isEqual(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Integer_Number.class){
			Integer_Number otherInteger = (Integer_Number)otherNumber;
			return value == otherInteger.value; 
		}else if(this.toDouble().equals(otherNumber)){
			return true;
		}
		//otherNumber type does not make sense
		throw new LogicException("trying check equality to an  integer number by a number type that does not make sense",MESSAGE_LEVEL.exception);

	}

	@Override
	/**
	 * TODO document
	 */
	public Integer_Number sqrt() {
		return new Integer_Number((int) Math.sqrt(value));
	}

	@Override
	/**
	 * TODO document
	 */
	public Integer_Number signum(){
		if(value >= 0){
			return new Integer_Number(1);
		}else{
			return new Integer_Number(-1);
		}
	}
	
	/**
	 * TODO document
	 * @return
	 */
	public Double_Number toDouble(){
		return new  Double_Number(value);
	}
	

	 
	 @Override
	 /**
	  * TODO document
	  */
	 public String toString(){
		return Integer.toString(value);
		 
	 }
	 
	 @Override
	 /**
	  * TODO document
	  */
	 public Integer_Number toInteger_Number(){
		 return this;
	 }
	 
	 @Override
	 /**
	  * Converts this integer_number to a double number, 
	  * NOTE: will have the same value  
	  * ie this.toDouble_Number().toIntgerNumber = this
	  */
	 public Double_Number toDouble_Number(){
		 return new Double_Number(value);
	 }

}
