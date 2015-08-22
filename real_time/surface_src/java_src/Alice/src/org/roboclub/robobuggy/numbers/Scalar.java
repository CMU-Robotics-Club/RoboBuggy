package org.roboclub.robobuggy.numbers;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;

/**
 * 
 * @author Trevor Decker
 * represents a unitless number
 *
 */
public abstract class Scalar implements Number{
	
	/**
	 * evaluates to a number whose value is squrt of this numbers value
	 * @throws CloneNotSupportedException 
	 */
	abstract public Number sqrt() throws CloneNotSupportedException;
	
	/**
	 * evaluates to the numbers one element if the number is positive or zero and inverse of one if the number is negative
	 * @throws CloneNotSupportedException 
	 */
	abstract public Number signum() throws CloneNotSupportedException;
	
	/**
	 * converts this number representation to a integer_number. if no such mapping exists or makes sense then a logic exception will be thrown
	 * @return
	 * @throws LogicException
	 */
	public Integer_Number toInteger_Number() throws LogicException{
		throw new LogicException("To Integer Number has not been implmented yet or does not make sense for this Number type",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * converts this number representation to a double_number. if no such mapping exists or makes sense then a logic exception will be thrown
	 * @return
	 * @throws LogicException
	 */
	public Double_Number toDouble_Number() throws LogicException{
		throw new LogicException("To Integer Number has not been implmented yet or does not make sense for this Number type",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * evaluates to the remainder of division of this number by someNumber
	 * @param someNumber
	 * @return
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	abstract public Number mod(Number someNumber) throws LogicException, CloneNotSupportedException;
	
}
