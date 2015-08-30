package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.measurements.Angle;

public class Rotation {
	Angle roationAmount;
	SpacialDimensions axisOfRotation;
	
	public Rotation(Angle rotationAmount,SpacialDimensions axisOfRotation){
		this.roationAmount = rotationAmount;
		this.axisOfRotation = axisOfRotation;
	}
}
