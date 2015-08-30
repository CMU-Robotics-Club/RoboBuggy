package org.roboclub.robobuggy.measurements;

import javax.print.attribute.standard.MediaSize.Other;

import org.omg.PortableInterceptor.DISCARDING;
import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.numbers.Double_Number;
import org.roboclub.robobuggy.numbers.Number;
import org.roboclub.robobuggy.numbers.Scalar;

/**
 * 
 * @author Trevor Decker
 * @version 0.0
 * object that represents a distance, encodes both a scalar along an axis of 
 * and the units of that measurement, can convert between different units and
 *  do math operations which will automatically convert units for you. 
 */
public class Distince extends  Measurement{
	//the internal representation of distance is in meters 
	//so all distance will be converted to meters
	
	
	/**
	 * TODO document
	 * @param meters
	 */
	private Distince(double meters){
		value = meters;
	}
	
	public String toString(){
		return toMeters() + " Meters";
	}
	
	/**
	 * TODO document
	 * @param meters
	 */
	public static Distince inch(double measurment){
		 return new Distince(0.0254*measurment);
	}

	/**
	 * TODO document
	 * @param meters
	 */
	public static Distince feet(double measurment){
		 return new Distince(0.3048*measurment);
	}
	
	/**
	 * TODO document
	 * @param meters
	 */
	public static Distince meter(double measurment){
		 return new Distince(measurment);
	}
	
	/**
	 * TODO document
	 * @param meters
	 */
	public static Distince centiMeter(double measurment){
		 return new Distince(0.01*measurment);
	}
	
	/**
	 * TODO document
	 * @param meters
	 */
	public static Distince kilometer(double measurment){
		 return new Distince(1000*measurment);
	}
	
	/**
	 * TODO document
	 * @param meters
	 */
	public static Distince mile(double measurment){
		 return new Distince(1609.34*measurment);
	}
	
	/**
	 * TODO document
	 * @param meters
	 */
	public double toMeters(){
		return value;
	}

	/**
	 * TODO document
	 * @param meters
	 */
	public double toInches(){
		return 39.3701*value;
	}
	
	/**
	 * TODO document
	 * @param meters
	 */
	public double toMiles(){
		return 0.000621371*value;
	}
	
	/**
	 * TODO document
	 * @param meters
	 */
	public double toCentiMeters(){
		return 100*value;
	}
	/**
	 * TODO document
	 * @param meters
	 */
	public double toKiloMeters(){
		return 1000*value;
	}
	
	public Distince multiply(double multiplyer){
		return new Distince(multiplyer*this.value);
	}
		
	public Area multiply(Distince otherDistince){
		return Area.metersSq(this.toMeters()*otherDistince.toMeters());
	}
	
	public Volume multiply(Area otherMeasurment){
		return Volume.metersCube(this.toMeters()*otherMeasurment.toMetersSq());
	}
	
	public AreaPerTime multiply(DistincePerTime otherMeasurment){
		return AreaPerTime.metersSquarePerSecond(this.toMeters()*otherMeasurment.toMetersPerSecond());
	}
	
	public VolumePerTime multiply(AreaPerTime otherMeasurment){
		return VolumePerTime.meterCubePerSecond(this.toMeters()*otherMeasurment.toMeterSquarePerSecond());
		}
	
	public DistincePerTime divide(Time time){
		return DistincePerTime.metersPerSecond(this.toMeters()/time.toSeconds());
	}
	
	
	public Distince divide(double scale){
		return new Distince(this.value/scale);
	}
	
	public double divide(Distince otherMeasurment){
		return this.value/otherMeasurment.value;
	}
	
	
}
