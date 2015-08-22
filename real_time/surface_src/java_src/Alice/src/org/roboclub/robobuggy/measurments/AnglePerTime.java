package org.roboclub.robobuggy.measurments;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.numbers.Number;

/**
 * 
 * @author Trevor Decker
 * A measurement object for encoding the rate at which an angle changes per time
 */
public class AnglePerTime extends Measurement{

	@Override
	/**
	 * TODO document
	 * TODO include
	 */
	public Number add(Number otherNumber) throws LogicException {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/**
	 * TODO document
	 * TODO include
	 */
	public Number sub(Number otherNumber) throws LogicException {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/**
	 * TODO document
	 * TODO include
	 */
	public Number mult(Number otherNumber) throws LogicException {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/**
	 * TODO document
	 * TODO include
	 */
	public Number div(Number otherNumber) {
		// TODO Auto-generated method stub
		return null;
	}

	/**
	 * TODO document
	 * TODO include 
	 * @return
	 */
	public static Number one() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/**
	 * TODO document
	 * TODO include
	 */
	public boolean isLess(Number someNumber) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	/**
	 * TODO document
	 * TODO include
	 */
	public boolean isGreater(Number someNumber) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	/**
	 * TODO document
	 * TODO include
	 */
	public boolean isEqual(Number somberNumber) throws LogicException {
		// TODO Auto-generated method stub
		return false;
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
