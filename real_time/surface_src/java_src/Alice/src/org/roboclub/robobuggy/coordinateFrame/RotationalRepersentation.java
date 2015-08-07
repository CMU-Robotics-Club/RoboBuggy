package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;

/**
 * TODO document 
 * @author Trevor Decker
 * @version 0.0
 *
 */
public interface RotationalRepersentation {
	/**
	 * TODO document
	 * @param secondRotation
	 * @return
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public RotationalRepersentation preApplyRotation(RotationalRepersentation secondRotation) throws LogicException, CloneNotSupportedException;
	
	/**
	 * TODO document
	 * @param secondRotation
	 * @return
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public RotationalRepersentation postApplyRotation(RotationalRepersentation secondRotation) throws LogicException, CloneNotSupportedException;

	/**
	 * TODO document
	 * @return
	 */
	public RotationMatrix toRotationMatrix();
	
	/**
	 * TODO document
	 * @return
	 */
	public EulerAngles toEulerAngles();
	
	/**
	 * TODO document
	 * @return
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public Quaternion toQuaternion() throws LogicException, CloneNotSupportedException;
	
	/**
	 * TODO document 
	 * @return
	 * @throws LogicException 
	 */
	public static RotationalRepersentation zero() throws LogicException {
		throw new LogicException("zero function has not been specified for this type of rotation", MESSAGE_LEVEL.exception);
	}
}
