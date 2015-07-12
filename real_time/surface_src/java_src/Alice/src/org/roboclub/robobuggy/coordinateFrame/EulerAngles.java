package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.linearAlgebra.ANGULAR_UNITS;
import org.roboclub.robobuggy.linearAlgebra.Angle;

/***
 * 
 * @author Trevor Decker
 * @version 0.0
 *
 * WARNING euler angles can reach a singularity when the y axis rotation reaches +- 90 degrees. 
 *  This is commonly known as gimble lock, to avoid the problem use RotationMatrices or Quaternions 
 */
public class EulerAngles implements RotationalRepersentation{
	private boolean inSingularity; //true if measurement reached a singularity and was not reset, else false
	private Angle[] angles = new Angle[3];
	//the order of the angles aka: xyx, yzy ...
	private SpacialDimensions[] Directions = new SpacialDimensions[3];

	
/***
 * Constructor for a euler angle, assumes that roll and pitch are both 0
 * @param yaw
 */
	/*
	public EulerAngles(Angle yaw){
		//roll and pitch assumed to be 0 
		this.yaw = yaw;
		this.roll = new Angle(ANGULAR_UNITS.DEGREES, 0);
		this.pitch = new Angle(ANGULAR_UNITS.RADIANS, 0);
		inSingularity = false;
	}
	*/
	
	/***
	 * Constructor for a representation of euler angles 
	 * @param roll
	 * @param pitch
	 * @param yaw
	 */
	/*
	public EulerAngles(Angle roll,Angle pitch,Angle yaw){
		this.roll = roll;
		this.pitch = pitch;
		this.yaw = yaw;
		inSingularity = false;
	}
	*/

	/***
	 * returns the value of the roll for this euler angle set
	 * @return
	 */
	/*
	public Angle getRoll(){
		return this.roll;
	}
	*/
	
	/***
	 * returns the value of the pitch for this euler angle set
	 * @return
	 */
	/*
	public Angle getPitch(){
		return this.pitch;
	}
	*/
	
	/***
	 * returns the value for the yaw for this euler angle set
	 * @return
	 */
	/*
	public Angle getYaw(){
		return this.yaw;
	}
	*/
	
	/***
	 * sets the value for the roll for this euler angle set
	 * @param newRoll
	 */
	/*
	public void setRoll(Angle newRoll){
		this.roll = newRoll;
	}
	*/
	
	/***
	 * sets the value for the pitch for this euler angle set
	 * @param newPitch
	 */
	/*
	public void setPitch(Angle newPitch){
		this.pitch = newPitch;
	}
	*/
	
	/***
	 * sets the value for the yaw for thsi euler angle set
	 * @param newYaw
	 */
	/*
	public void setYaw(Angle newYaw){
		this.yaw = newYaw;
	}
	*/
	
	/**
	 * TODO document
	 * @return
	 */
	public boolean inSingularity(){
		return inSingularity;
	}
	
	/**
	 * TODO document
	 */
	public void setInSingularity(){
		inSingularity = true;
		}
	
	/**
	 * TODO document 
	 * TODO implement
	 * @return
	 */
	public RotationMatrix toRotationMatrix(){
		return null;
		//TODO
	}
	
	/**
	 * TODO document
	 * TODO implement
	 * @return
	 */
	public Quaternion toQuaternion(){
		return null;
		//TODO
	}

	@Override
	/**
	 * TODO document
	 * TODO implement
	 */
	public RotationalRepersentation preApplyRotation(
			RotationalRepersentation secondRotation) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/**
	 * TODO document
	 */
	public EulerAngles toEulerAngles() {
		return this;
	}

	@Override
	/**
	 * TODO implement
	 * TODO document
	 */
	public RotationalRepersentation postApplyRotation(
			RotationalRepersentation secondRotation) {
		// TODO Auto-generated method stub
		return null;
	}
	
	
}
