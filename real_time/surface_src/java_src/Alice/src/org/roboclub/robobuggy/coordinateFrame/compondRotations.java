package org.roboclub.robobuggy.coordinateFrame;

import java.util.ArrayList;

import org.roboclub.robobuggy.measurments.Angle;


/**
 * Represents a string of rotations being applied to create a net rotation 
 * Euler angles would be a specific ordering of thease rotations 
 * be carful that singularits can exsits with this orintation repersentation 
 * @author Trevor Decker
 *
 */
public class CompondRotations {
	ArrayList<Rotation> rotations;
	
	//creates a compondRotation that only has a single rotation about 1 axis
	public CompondRotations(Angle rotationAmount,SpacialDimensions axisOfRotation){
		rotations = new ArrayList<Rotation>();
		rotations.add(new Rotation(rotationAmount,axisOfRotation));
	}
	
	public CompondRotations(Rotation aRotation){
		rotations = new ArrayList<Rotation>();
		rotations.add(aRotation);
	}
	
	//creates a seieres of compond Rotations based on intput array of rotation_amounts, and rotation_axis
	public CompondRotations(ArrayList<Rotation> rotations){
		this.rotations = rotations;
	}

	//warning has side effects
	public void postApplyRotation(CompondRotations otherRotation){
		this.rotations.addAll(otherRotation.rotations);
	}
	
	//this method does not have side effects
	public CompondRotations preApplyRotation(CompondRotations otherRotation) throws CloneNotSupportedException{
		ArrayList<Rotation> R1 = (ArrayList<Rotation>) otherRotation.clone();
		ArrayList<Rotation> R2 = (ArrayList<Rotation>) this.rotations.clone();
		R1.addAll(R2);
		return new CompondRotations(R1);
	}
	
	
	
	//TODO add clone
	//TODO remove a rotation 
	//TODO change a rotation 
	
	//TODO apply rotations to something  (a vector)
	//TODO inverse rotations (TODO)
	
}
