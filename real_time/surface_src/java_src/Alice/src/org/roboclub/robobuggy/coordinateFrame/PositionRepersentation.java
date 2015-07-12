package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.linearAlgebra.Distince;
import org.roboclub.robobuggy.linearAlgebra.Vector;
import org.roboclub.robobuggy.main.LogicException;

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
	 * TODO document
	 * @return
	 */
	public Distince getDistince();
	
	/**
	 * TODO document
	 * @return
	 */
	public SpacialDimensions[] getDimensions();
	
	public static PositionRepersentation zero() {
		//TODO
		return null;
	}
}
