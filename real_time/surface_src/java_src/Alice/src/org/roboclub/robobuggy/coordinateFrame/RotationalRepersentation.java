package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.main.LogicException;

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
	 */
	public RotationalRepersentation preApplyRotation(RotationalRepersentation secondRotation) throws LogicException;
	
	/**
	 * TODO document
	 * @param secondRotation
	 * @return
	 * @throws LogicException 
	 */
	public RotationalRepersentation postApplyRotation(RotationalRepersentation secondRotation) throws LogicException;

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
	 */
	public Quaternion toQuaternion() throws LogicException;
	
	/**
	 * TODO document 
	 * @return
	 */
	public static RotationalRepersentation zero() {
		//TODO implment 
		return null;
	}
}
