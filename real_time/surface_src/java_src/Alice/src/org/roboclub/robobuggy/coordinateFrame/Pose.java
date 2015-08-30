package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.measurements.Angle;
import org.roboclub.robobuggy.measurements.Distince;


/**
 * 
 * @author Trevor Decker
 * @version 0.0
 * TODO document
 */
public class Pose implements FrameOfRefrence{
	private PositionRepersentation position;
	private RotationalRepersentation orintation;
	
	/**
	 * TODO document 
	 * @param position
	 * @param orintation
	 */
	public Pose(PositionRepersentation position,RotationalRepersentation orintation){
		this.position = position;
		this.orintation = orintation;
	}
	

	/**
	 * TODO document 
	 * TODO implement
	 * @return
	 */
	public static Pose zero(){
		//TOOD may need to set the size and type of position/orintation, but ideally the system will figure it out on its own 

		return null;
	}
	
	/**
	 * TODO document
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public FrameOfRefrence preApply(FrameOfRefrence otherFrame) throws LogicException, CloneNotSupportedException{
		//rotate then translate 
		RotationalRepersentation newOrintation = this.orintation.preApplyRotation(otherFrame.getOrintation());
		PositionRepersentation newPosition = position.applyRotation(otherFrame.getOrintation()).add(otherFrame.getPosition());
		return new Pose(newPosition, newOrintation); 
	}
	
	/**
	 * TODO document
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public FrameOfRefrence postApply(FrameOfRefrence otherFrame) throws LogicException, CloneNotSupportedException{
		return otherFrame.preApply(this);
	}
	
	/**
	 * TODO document
	 * @return
	 */
	public PositionRepersentation getPosition(){
		return position;
	}
	
	/**
	 * TODO document
	 * @return
	 */
	public RotationalRepersentation getOrintation(){
		return orintation;
	}

	/**
	 * TODO document 
	 * TODO implement
	 */
	public FrameOfRefrence inverse(){
		return null;
		//TODO
	}
	
	@Override
	/**
	 * TODO document
	 * TODO implement
	 */
	public HomogeneousMatrix toHomogeneousMatrix() {
		return null;
	}


	@Override
	/**
	 * TODO document
	 */
	public Pose toPose() {
		return this;
	}


	/**
	 * TODO document
	 * TODO implement
	 * @param refrenceFrame
	 * @return
	 */
	public boolean isGreater(Pose refrenceFrame) {
		// TODO Auto-generated method stub
		return false;
	}

	/**
	 * TODO document
	 * TODO implement
	 * @param refrenceFrame
	 * @return
	 */
	public boolean isLess(Pose refrenceFrame) {
		// TODO Auto-generated method stub
		return false;
	}


}
