package org.roboclub.robobuggy.measurments;

import org.roboclub.robobuggy.numbers.Scalar;
import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.numbers.Number;

/**
 * TODO document
 * @author Trevor Decker
 *
 */
public abstract class Measurement implements Number{
	double value;	
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
	public boolean isLess(Number someNumber) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isGreater(Number someNumber) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean isEqual(Number somberNumber) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}
}
