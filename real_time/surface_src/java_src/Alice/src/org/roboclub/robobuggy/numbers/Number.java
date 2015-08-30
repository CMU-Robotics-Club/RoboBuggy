package org.roboclub.robobuggy.numbers;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;

/**
 *
 * @author Trevor Decker
 * @version 0.0
 * Computer representation of a Number, a Number can be a very abstract 
 * concept but needs to meet this interface
 */
public interface Number {
	/**
	 * evaluates to the inverse representation of the number
	 * ie Number + Number.inverse() = 0 
	 * 	@return inverse of calling number
	 * @throws CloneNotSupportedException 
	 * @throws LogicException 
	 */
	default public Number inverse() throws CloneNotSupportedException, LogicException{
		 throw new LogicException("This function needs to be over written",MESSAGE_LEVEL.exception);
	}
	
	/**
	 * returns the result of adding this number to some otherNumber
	 * should be commutative and have closure
	 * If the two numbers are of different types and no conversion  exists between them
 	 * then a logic error will be raised
	 * @param otherNumber
	 * @return
	 * @throws LogicException 
	 */
	 public default Number add(Number otherNumber) throws LogicException{
		 throw new LogicException("This function needs to be over written",MESSAGE_LEVEL.exception);
	 }
	
	/**
	 * returns the result of subtracting some otherNumber from this number
	 * If two numbers are of different types and no conversion exists between them 
	 * then a logic error will be raised
	 * @param otherNumber
	 * @return
	 * @throws LogicException 
	 */
	 public default Number sub(Number otherNumber) throws LogicException{
		 throw new LogicException("This function needs to be over written",MESSAGE_LEVEL.exception);
	 }
	
	/**
	 * returns the result of multiplying some otherNumber from this number
	 * If two numbers are of different types and no conversion exists between them 
	 * then a logic error will be raised
     * @param otherNumber
	 * @return
	 * @throws LogicException 
	 */
	 public default Number mult(Number otherNumber) throws LogicException{
		 throw new LogicException("This function needs to be over written",MESSAGE_LEVEL.exception);
	 }
	
	/**
	 * returns the result of dividing some otherNumber from this number
	 * If two numbers are of different types and no conversion exists between them 
	 * then a logic error will be raised	
	 * @param otherNumber
	 * @return
	 * @throws LogicException 
	 */
	 public default Number div(Number otherNumber) throws LogicException{
		 throw new LogicException("This function needs to be over written",MESSAGE_LEVEL.exception);
	 }
	
	/**
	 * Evaluates to this number types zero element
	 * @return
	 * @throws LogicException 
	 */
	 public static Number zero() throws LogicException {
		 throw new LogicException("This function needs to be over written",MESSAGE_LEVEL.exception);

	}
	 
	/**
	 * Evaluates to this number types one element
	 * @return
	 * @throws LogicException 
	 */
	 public static Number one() throws LogicException {
		 throw new LogicException("This function needs to be over written",MESSAGE_LEVEL.exception);
	}


	/**
	 * evaluates to true if this number represents a number less then some otherNumber
	 * otherwise evaluates to false.  if the comparison of number types does not make 
	 * sense then a logicException will be thrown
	 * @param someNumber
	 * @return
	 * @throws LogicException 
	 */
	abstract public boolean isLess(Number someNumber) throws LogicException;
	
	/**
	/**
	 * evaluates to true if this number represents a number greater then some otherNumber
	 * otherwise evaluates to false.  if the comparison of number types does not make 
	 * sense then a logicException will be thrown
	 * @param someNumber
	 * @return
	 * @throws LogicException 
	 */
	abstract public boolean isGreater(Number someNumber) throws LogicException;
	
	/**
	 * Evaluates to true if some other Number is of the same value and type as this Double number
	 * currently will only evaluate to true iff the otherNumber is a Double number of the exact 
	 * same value as this Double number
	 * @param somberNumber
	 * @return
	 * @throws LogicException 
	 */
	abstract public boolean isEqual(Number somberNumber) throws LogicException;
	
	/**
	 * Evaluates to the number systems zero value
	 * @return
	 * @throws LogicException
	 */
	public default Number getZero() throws LogicException{
		return zero();
	}
	
	/**
	 * Evaluates to the number systems one value (identity value) 
	 * @return
	 * @throws LogicException
	 */
	public default Number getOne() throws LogicException{
		return one();
	}

}
