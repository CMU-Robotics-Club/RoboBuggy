package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.linearAlgebra.Vector;
import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.measurments.Distince;

/**
 * TODO document
 * @author Trevor Decker
 * @version 0.0
 *
 */
public interface PositionRepersentation {
	
	
	
	/**
	 * TODO document 
	 * @param direction
	 * @param newValue
	 * @throws LogicException 
	 */
	public void setDimension(SpacialDimensions direction,Distince newValue) throws LogicException;
	
	/**
	 * TODO document 
	 * @param direction
	 * @return
	 * @throws LogicException 
	 */
	public Distince getDimension(SpacialDimensions direction) throws LogicException;
	
	/**
	 * TODO document 
	 * @param index
	 * @return
	 * @throws LogicException 
	 */
	public Distince getDimensionIndex(int index) throws LogicException;
	
	/**
	 * TODO document
	 * @param R
	 * @return
	 */
	public PositionRepersentation applyRotation(RotationalRepersentation R);
	
	/**
	 * TODO document
	 * @param T
	 * @return
	 */
	public PositionRepersentation add(PositionRepersentation T);
	

	/**
	 * evaluates to the distance between this position representation and the origin of the frame
	 * @return
	 */
	public Distince getDistince();
	
	/**
	 * TODO document
	 * @return
	 */
	public SpacialDimensions[] getDimensions();
	
	/**
	 * TODO document
	 * @return
	 * @throws LogicException 
	 */
	public static PositionRepersentation zero() throws LogicException {
		throw new LogicException("zero has not yet been defined for this type of position", MESSAGE_LEVEL.exception);
	}
}
