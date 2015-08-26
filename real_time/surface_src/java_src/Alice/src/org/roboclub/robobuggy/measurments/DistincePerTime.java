package org.roboclub.robobuggy.measurments;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.numbers.Number;

/**
 * 
 * @author Trevor Decker
 * A measurement object for encoding the rate at which a distance changes per time
 */
public class DistincePerTime extends Measurement{
	//the internal measurment of distincePerTime unit is meters per seconds
	
	private DistincePerTime(double input){
		value = input;
	}
	
	public static DistincePerTime metersPerSecond(double input){
		return new DistincePerTime(input);
	}
	
	public double toMetersPerSecond(){
		return	value;
	}
	
	public String toString(){
		return toMetersPerSecond() + " Meters per Second";
	}
	
	public DistincePerTime add(DistincePerTime otherMeasurment){
		return new DistincePerTime(this.value + otherMeasurment.value); 	
	}
	
	public DistincePerTime subtract(DistincePerTime otherMeasurment){
		return new DistincePerTime(this.value - otherMeasurment.value);
	}
	
	public DistincePerTime multiply(double scale){
		return new DistincePerTime(this.value*scale);
	}
	
	public DistincePerTime divide(double scale){
		return new DistincePerTime(this.value/scale);
	}
	
	public double divide(DistincePerTime otherMeasurment){
		return this.value/otherMeasurment.value;
	}
	
	public Time divide(Distince otherMeasurment){
		double frequency =  this.toMetersPerSecond()/otherMeasurment.toMeters();
		return Time.second(1/frequency);
	}
	
	public AreaPerTime multiply(Distince otherMeasurment){
		return AreaPerTime.metersSquarePerSecond(this.toMetersPerSecond()*otherMeasurment.toMeters());
	}
	
	public VolumePerTime multiply(Area otherMeasurment){
		return VolumePerTime.meterCubePerSecond(this.toMetersPerSecond()*otherMeasurment.toMetersSq());
	}

}
