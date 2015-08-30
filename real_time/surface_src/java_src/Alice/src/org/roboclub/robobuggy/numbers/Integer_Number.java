package org.roboclub.robobuggy.numbers;

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
public class Integer_Number extends Scalar{
	private int value;
	/**
	 * Constructs an object as a integer Number. 
	 * A double which implements the Number class. 
	 *  This is so that the number can be used with code designed for the number interface
	 *  @param value : the value for the integer that this number represents
	 */
	public Integer_Number(int numberValue) {
		this.value = numberValue;
	}

	/**
	 * Evaluates to a number representation of this number added to the other number, 
	 * order of addition should not matter
	 */
	public Integer_Number add(Scalar otherNumber) throws LogicException {
		return new Integer_Number(value+(otherNumber.toInteger_Number()).getValue());
	}
	
	/**
	 * returns the current value of this number 
	 * @return
	 */
	public int getValue(){
		return value;
	}

	@Override
	/**
	 * Subtracts this numbers value by the other numbers value 
	 * @param Number other Number
	 */
	public Integer_Number sub(Number otherNumber) {
		return  new Integer_Number(value-((Integer_Number) otherNumber).getValue());
	}

	@Override
	/**
	 * Multiplies this numbers value by the otherNumbers value 
	 * @param Number other Number 
	 */
	public Integer_Number mult(Number otherNumber) {
		return  new Integer_Number(value*((Integer_Number) otherNumber).getValue());
	}

	@Override
	/**
	 * Divides this numbers value by the otherNumbers value 
	 * @param Number otherNumber
	 */
	public Integer_Number div(Number otherNumber) {
		return new Integer_Number(value/((Integer_Number) otherNumber).getValue());
	}

	/**
	 * Creates a Integer number representing zero
	 */
	public static Integer_Number zero() {
		return new Integer_Number(0);
	}
	
	/**
	 * Creates a Integer number representing of the number 1
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
	 * Evaluates to a new integer_number whos value is this number mod some Number
	 * If a conversion between this number's type and someNuber's type does not exist 
	 * then a logic exception will be thrown 	 
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
	 * evaluates to true if this number represents a number less then some otherNumber
	 * otherwise evaluates to false.  if the comparison of number types does not make 
	 * sense then a logicException will be thrown
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
	 * evaluates to true if this number represents a number greater then some otherNumber
	 * otherwise evaluates to false.  if the comparison of number types does not make 
	 * sense then a logicException will be thrown
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
	 * Evaluates if some other Number is of the same value and type as this Double number
	 * currently will only evaluate to true iff the otherNumber is a integer number of the exact 
	 * same value as this integer number
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
	 * evaluates to a integer number whose value is a close approximation of the squrt of this numbers value
	 */
	public Integer_Number sqrt() {
		return new Integer_Number((int) Math.sqrt(value));
	}

	@Override
	/**
	 * Evaluates to a representation of 1 if the number's value is greater then
	 * or equal to 0 and evaluates to a representation of -1 if the numbers value is
	 * less then 0
	 */
	public Integer_Number signum(){
		if(value >= 0){
			return new Integer_Number(1);
		}else{
			return new Integer_Number(-1);
		}
	}
	
	/**
	 * Converts this number to a double_number of the same value
	 * @return
	 */
	public Double_Number toDouble(){
		return new  Double_Number(value);
	}
	

	 
	 @Override
	 /**
	  * evaluates to a string which displays the information encoded in this number
	  */
	 public String toString(){
		return Integer.toString(value);
		 
	 }
	 
	 @Override
	 /**
	 * Converts this number to a Integer_number of the same value, here for completeness and use by abstract classes
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
	 
		@Override
		/**
		 * Overides the classes default equal function to fit the number equal function 
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
