package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.linearAlgebra.Angle;
import org.roboclub.robobuggy.linearAlgebra.Distince;
import org.roboclub.robobuggy.main.LogicException;

/**
 * @version 0.0
 * @author Trevor Decker
 * TODO document
 */
public interface FrameOfRefrence{
	/**
	 * TODO document
	 * @param otherFrame
	 * @return
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public abstract FrameOfRefrence preApply(FrameOfRefrence otherFrame) throws LogicException, CloneNotSupportedException;
	
	/**
	 * TODO document
	 * @param otherFrame
	 * @return
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public abstract FrameOfRefrence postApply(FrameOfRefrence otherFrame) throws LogicException, CloneNotSupportedException;
	
	/**
	 * TODO document
	 * @return
	 * @throws LogicException 
	 */
	public abstract FrameOfRefrence inverse() throws LogicException;
	
	/**
	 * TODO document
	 * @return
	 */
	public abstract RotationalRepersentation getOrintation();
	
	/**
	 * TODO document
	 * @return
	 */
	public abstract PositionRepersentation getPosition();
	
	/**
	 * TODO document
	 * @return
	 */
	public abstract HomogeneousMatrix toHomogeneousMatrix();

	/**
	 * TODO document
	 * @return
	 */
	public abstract Pose toPose();


	
	

}
