package org.roboclub.robobuggy.map;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.measurments.Distince;

/**
 * TODO document 
 * @author Trevor Decker
 *
 * @param <NTYPE>
 */
public class Coefficnet<NTYPE extends Number> {
	protected NTYPE value;			//the value of this exponent 
	protected NTYPE exponent;		//the power to which this coefficent should be raised 
	
	
	/**
	 * TODO document
	 */
	public Coefficnet(NTYPE value,NTYPE exponent) {
		this.value = value;
		this.exponent = exponent;
	}
	
	/**
	 * TODO document
	 * @param obj
	 * @return
	 * @throws LogicException 
	 */
	boolean Equals(Object obj) throws LogicException {
		if(obj.getClass() == Coefficnet.class){
			//TODO fix implicit assumption that other obj is of number type NTYPE
			Coefficnet<NTYPE> otherCoefficent = (Coefficnet<NTYPE>)obj; 
			return this.value.equals(otherCoefficent.value);
		}
		throw new LogicException("It does not make sense to check equality of objects of diffrent types", MESSAGE_LEVEL.exception);
	}

	/**
	 * TODO document
	 * TODO implement
	 * @param obj
	 * @return
	 * @throws LogicException
	 */
	boolean isGreater(Object obj) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * TODO document
	 * TODO implement
	 * @param obj
	 * @return
	 * @throws LogicException
	 */
	boolean isLess(Object obj) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}
}
