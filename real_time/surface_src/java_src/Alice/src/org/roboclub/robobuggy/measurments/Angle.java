package org.roboclub.robobuggy.measurments;

import java.io.ObjectInputStream.GetField;
import java.lang.reflect.InvocationTargetException;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.numbers.Double_Number;
import org.roboclub.robobuggy.numbers.Number;

/**
 * @author Trevor Decker
 * TODO document
 * @version 0.0
 * 
 */
public class Angle extends Measurement{
	//the internal measure of angle is in radians
	
	private Angle(double input){
		value = input;
	}
	
	public static Angle radins(double input){
		return new Angle(input);
	}
	public static Angle degrees(double input){
		return new Angle(Math.PI*input/180);
	}
	
	public double toRadians(){
		return value;
	}
	
	public double toDegrees(){
		return 180*value/Math.PI;
	}
	
	public String toString(){
		return toRadians() + " radians";
	}
	
	public Angle add(Angle otherAngle){
		return new Angle(this.value + otherAngle.value);
	}
	
	public Angle sub(Angle otherAngle){
		return new Angle(this.value - otherAngle.value);
	}
	
	public Angle mult(double scale){
		return new Angle(this.value*scale);
	}
	
	public Angle div(double scale){
		return new Angle(this.value/scale);
	}
	
	public double div(Angle otherAngle){
		return this.value / otherAngle.value;
	}
	
	public AnglePerTime div(Time t){
		return AnglePerTime.radianPerSecond(this.toRadians()/t.toSeconds());
	}

}
