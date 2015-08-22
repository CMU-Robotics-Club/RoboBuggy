package org.roboclub.robobuggy.numbers;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;

/**
 *
 * @author Trevor Decker
 * @version 1.0
 *
 * A double which implements the Number class. 
 * This is so that the number can be used with 
 * code designed for the number interface
 */
public class Double_Number extends Scalar {
	private double value;
	
	/**
	 * Constructs an object as a Double Number. 
	 * A double which implements the Number class. 
	 *  This is so that the number can be used with code designed for the number interface
	 *  @param value : the value for the double that this number represents
	 */
	public Double_Number(double value){
		this.value = value;
	}
	
	/**
	 * returns the current value of this number 
	 * @return
	 */
	public double getValue(){
		return value;
	}
	
	/**
	 * returns a new number representing this number added to otehrNumber
	 * @param otherNumber
	 */
	public Number add(Number otherNumber) {
		return (Number) new Double_Number(value+((Double_Number) otherNumber).getValue());
	}

	/**
	 * Subtracts this numbers value by the other numbers value 
	 * @param otherNumber
	 */
	@Override
	public Number sub(Number otherNumber) {
		return (Number) new Double_Number(value+((Double_Number) otherNumber).getValue());
	}

	/**
	 * Multiplies this numbers value by the otherNumbers value 
	 * @param otherNumber
	 */
	@Override
	public Number mult(Number otherNumber) {
		return (Number) new Double_Number(value*((Double_Number) otherNumber).getValue());
	}

	/**
	 * Divides this numbers value by the otherNumbers value 
	 * @param otherNumber
	 */
	@Override
	public Number div(Number otherNumber) {
		return (Number) new Double_Number(value/((Double_Number) otherNumber).getValue());
	}
	
	/**
	 * converts this Number to an integer, note that this will cause a loss of perscion. 
	 * this.toIntegr_Number().toDouble_Number == this iff this.value is an integer  
	 * @return
	 */
	public Integer_Number toInteger_Number(){
		return new Integer_Number((int) this.getValue()); 
	}

	
	/**
	 * Creates a Double number representing zero
	 */
	public static Double_Number zero() {
		return new Double_Number(0.0);
	}
	
	/**
	 * Creates a Double number representing the identy element 1.0
	 */
	public static Double_Number one(){
		return new Double_Number(1.0);
	}

	@Override
	/**
	 * evaluates to the inverse representation of the number,
	 * in this case -1*the calling objects value
	 */
	public Double_Number inverse() {
		return new Double_Number(-value);
	}

	@Override
	/**
	 * Evaluates to a new double_number whos value is this number mod some Number
	 * If a conversion between this number's type and someNuber's type does not exist 
	 * then a logic exception will be thrown 
	 */
	public Double_Number mod(Number someNumber) throws LogicException {
		if(someNumber.getClass() == Integer_Number.class){
			Integer_Number otherInteger = (Integer_Number)someNumber;
			return new Double_Number(value % otherInteger.getValue());
		}		
		
		if(someNumber.getClass() == Double_Number.class){
			Double_Number otherDouble = (Double_Number)someNumber;
			return new Double_Number(value % otherDouble.value);
		}
		
		throw new LogicException("trying to mod a double number by a number type that does not make sense",MESSAGE_LEVEL.exception);

	}

	@Override
	/**
	 * evaluates to true if this number represents a number less then some otherNumber
	 * otherwise evaluates to false.  if the comparison of number types does not make 
	 * sense then a logicException will be thrown
	 */
	public boolean isLess(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Double_Number.class){
			Double_Number otherDouble = (Double_Number)otherNumber;
			return value < otherDouble.value; 
		}else if(this.toInteger_Number().isLess(otherNumber)){
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
	 */
	public boolean isGreater(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Double_Number.class){
			Double_Number otherDouble = (Double_Number)otherNumber;
			return value > otherDouble.value; 
		}else if(this.toInteger_Number().isGreater(otherNumber)){
			return true;
		}
		//otherNumber type does not make sense
		throw new LogicException("trying check equality to an  integer number by a number type that does not make sense",MESSAGE_LEVEL.exception);
	}

	@Override
	/**
	 * Evaluates to true if some other Number is of the same value and type as this Double number
	 * currently will only evaluate to true iff the otherNumber is a Double number of the exact 
	 * same value as this Double number
	 */
	public boolean isEqual(Number otherNumber) throws LogicException {
		if(otherNumber.getClass() == Double_Number.class){
			Double_Number otherDouble = (Double_Number)otherNumber;
			return value == otherDouble.value; 
		}else if(this.toInteger_Number().equals(otherNumber)){
			return true;
		}
		//otherNumber type does not make sense
		throw new LogicException("trying check equality to an  integer number by a number type that does not make sense",MESSAGE_LEVEL.exception);
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

	@Override
	/**
	 * evaluates to a double number whose value is a close approximation of the squrt of this numbers value
	 */
	public Double_Number sqrt() {
		return new Double_Number(Math.sqrt(value));
	}

	@Override
	/**
	 * Evaluates to a representation of 1.0 if the number's value is greater then
	 * or equal to 0.0 and evaluates to a representation of -1.0 if the numbers value is
	 * less then 0.0 
	 */
	public Double_Number signum() {
		if(value >= 0.0){
			return new Double_Number(1.0);
		}else{
			return new Double_Number(-1.0);
		}
	}
	
	/**
	 * Converts this number to an integer by taking the floor of the "double's" value
	 * @return
	 */
	public Integer_Number toInteger(){
		return new Integer_Number((int) Math.floor(value));
	}
	

	
}
