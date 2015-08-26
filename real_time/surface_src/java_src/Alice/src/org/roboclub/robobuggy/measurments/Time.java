package org.roboclub.robobuggy.measurments;

import javax.print.attribute.standard.MediaSize.Other;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.numbers.Number;

public class Time extends Measurement{
	//The internal repersentation of time is in seconds 
	
	
	private Time(double seconds){
		value = seconds;
	}
	
	public static Time second(double seconds){
		return new Time(seconds);
	}
	
	public double toSeconds(){
		return value;
	}
	
	public String toString(){
		return toSeconds() + " Seconds";
	}

	public Time add(Time otherTime){
		return new Time(this.value + otherTime.value);
	}
	
	public Time subtract(Time otherTime){
		return new Time(this.value - otherTime.value);
	}
	
	public Time multiply(double scale){
		return new Time(this.value*scale);
	}
	
	public Time divide(double scale){
		return new Time(this.value/scale);
	}
	
	public double divide(Time otherTime){
		return this.value/otherTime.value;
	}
	
	public Angle multiply(AnglePerTime apt){
		return Angle.radins(this.toSeconds()*apt.toRadianPerSecond());
	}
	
	public Area multiply(AreaPerTime apt){
		return Area.metersSq(this.toSeconds()*apt.toMeterSquarePerSecond());
	}
	
	public Distince multiply(DistincePerTime dpt){
		return Distince.meter(this.toSeconds()*dpt.toMetersPerSecond());
	}
	
	public Volume multiply(VolumePerTime vpt){
		return Volume.metersCube(this.toSeconds()*vpt.toMeterCubePerSecond());
	}
	
	
}
