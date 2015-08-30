package org.roboclub.robobuggy.measurements;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.numbers.Number;

/**
 * 
 * @author Trevor Decker
 * A measurement object for encoding the rate at which an angle changes per time
 */
public class AnglePerTime extends Measurement{
	//internal repersentation of anlge per time is radian per second

	private AnglePerTime(double input){
		value = input;
	}
	
	public static AnglePerTime radianPerSecond(double input){
		return new AnglePerTime(input);
	}
	
	public static AnglePerTime degreePerSecond(double input){
		return new AnglePerTime(Math.PI*input/180);
		}
	
	public double toDegreePerSecond(){
		return 180*value/Math.PI;
	}
	
	public double toRadianPerSecond(){
		return value;
	}
	
	public String toString(){
		return this.toDegreePerSecond() + "Degrees Per Second";
	}
	
	public AnglePerTime add(AnglePerTime measurment){
		return new AnglePerTime(this.value + measurment.value);
	}
	
	public AnglePerTime sub(AnglePerTime measurment){
		return new AnglePerTime(this.value - measurment.value);
	}
	
	public AnglePerTime mult(double scale){
		return new AnglePerTime(this.value*scale);
	}
	
	public AnglePerTime div(double scale){
		return new AnglePerTime(this.value/scale);
	}
	
	public double div(AnglePerTime measurment){
		return this.value/ measurment.value;
	}
	
	public Angle mult(Time measurment){
		return Angle.radins(toRadianPerSecond()*measurment.toSeconds());
		
	}

}
