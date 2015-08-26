package org.roboclub.robobuggy.measurments;

/**
 * 
 * @author Trevor Decker
 * TODO document 
 */
public class Area extends Measurement{
	//internal variable value is in metersSq
	
	private Area(double metersSq){
		value = metersSq;
	}
	
	public String toString(){
		return toMetersSq() + " Meters^2";
	}
	
	static public Area metersSq(double metersSq){
		return new Area(metersSq);
	}
	
	static public Area inchesSq(double inchesSq){
		return new Area(inchesSq/1550.0);
	}
	
	static public Area feet_sq(double feetSq){
		return new Area(feetSq/10.764);
	}
	
	public double toMetersSq(){
		return value;
	}
	
	public double toFeetSq(){
		return 10.764*value;
	}
	
	public double toInchesSq(){
		return 1550.0*value;
	}
	
	public Volume mult(Distince otherDistince){
		return Volume.metersCube(this.toMetersSq()*otherDistince.toMeters());
	}
	
	public Area mult(double scale){
		return new Area(scale*this.value);
	}
	
	public double div(Area otherArea){
		return this.value/otherArea.value;
	}
	
	public Distince div(Distince otherDistince){
		return Distince.meter(this.toMetersSq()/otherDistince.toMeters());
	}
	
	
	public AreaPerTime div(Time someTime){
		return AreaPerTime.metersSquarePerSecond(toMetersSq()/someTime.toSeconds());
	}
	
	public VolumePerTime mult(DistincePerTime otherMeasurment){
		return VolumePerTime.meterCubePerSecond(this.toMetersSq()*otherMeasurment.toMetersPerSecond());
	}
	
	public Area add(Area otherArea){
		return new Area(this.value + otherArea.value);
	}
	
	public Area sub(Area otherArea){
		return new Area(this.value - otherArea.value);
	}	

}
