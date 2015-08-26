package org.roboclub.robobuggy.measurments;

public class AreaPerTime extends Measurement{
	//the internal value for area/Time is in meters^2/second
	
	private AreaPerTime(double measurment){
		value = measurment;
	}
	
	public static AreaPerTime metersSquarePerSecond(double measurment){
		return new AreaPerTime(measurment);
	}
	
	public double toMeterSquarePerSecond(){
		return this.value;
	}
	
	public AreaPerTime add(AreaPerTime otherMeasurment){
		return new AreaPerTime(value+otherMeasurment.value);
	}
	
	public AreaPerTime sub(AreaPerTime otherMeasurment){
		return new AreaPerTime(value-otherMeasurment.value);
	}
	
	public AreaPerTime mult(double scale){
		return new AreaPerTime(value*scale);
	}
	
	public AreaPerTime div(double scale){
		return new AreaPerTime(value/scale);
	}
	
	public double div(AreaPerTime otherMeasurment){
		return this.toMeterSquarePerSecond()/otherMeasurment.toMeterSquarePerSecond();
	}
	
	public DistincePerTime div(Distince otherMeasurment){
		return DistincePerTime.metersPerSecond(this.toMeterSquarePerSecond()/otherMeasurment.toMeters());		
	}
	
	public Distince div(DistincePerTime otherMeasurment){
		return Distince.meter(this.toMeterSquarePerSecond()/otherMeasurment.toMetersPerSecond());
		
	}
	
	public VolumePerTime mult(Distince otherMeasurment){
		return  VolumePerTime.meterCubePerSecond(this.toMeterSquarePerSecond()*otherMeasurment.toMeters());		
	}
	
		
	
}
