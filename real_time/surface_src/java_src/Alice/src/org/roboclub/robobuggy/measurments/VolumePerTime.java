package org.roboclub.robobuggy.measurments;

public class VolumePerTime extends Measurement{
	//the internal default unit for volume per time is meter^3 per second
	
	private VolumePerTime(double meterCubePerSecond){
		value = meterCubePerSecond;
	}
	
	public static VolumePerTime meterCubePerSecond(double measurment){
		return new VolumePerTime(measurment);
	}
	
	public double toMeterCubePerSecond(){
		return value;
	}
	
	public String toString(){
		return toMeterCubePerSecond() + "Meters^3/second";
	}
	
	public VolumePerTime add(VolumePerTime otherMeasurment){
		return new VolumePerTime(otherMeasurment.value + this.value);
	}
	
	public VolumePerTime sub(VolumePerTime otherMeasurment){
		return new VolumePerTime(otherMeasurment.value - this.value);
	}
	
	public VolumePerTime mult(double scale){
		return new VolumePerTime(this.value*scale);
	}
	
	public VolumePerTime div(double scale){
		return new VolumePerTime(this.value/scale);
	}
	
	public Volume mult(Time t){
		return Volume.metersCube(this.toMeterCubePerSecond()*t.toSeconds());
	}
	
	public double div(VolumePerTime otherMeasurment){
		return this.value/otherMeasurment.value;
	}
	
	public Time div(Volume otherMeasurment){
		double frequency = this.toMeterCubePerSecond()/otherMeasurment.toMetersCube();
		return Time.second(1/frequency);
	}
	
	public DistincePerTime div(Area otherMeasurment){
		return DistincePerTime.metersPerSecond(this.toMeterCubePerSecond()/otherMeasurment.toMetersSq());
	}
	
	public AreaPerTime div(Distince otherMeasurment){
		return AreaPerTime.metersSquarePerSecond(this.toMeterCubePerSecond()/otherMeasurment.toMeters());
	}
	
	public Distince div(AreaPerTime otherMeasurment){
		return Distince.meter(this.toMeterCubePerSecond()/otherMeasurment.toMeterSquarePerSecond());
	}
	
	public Area div(DistincePerTime otherMeasurment){
		return Area.metersSq(this.toMeterCubePerSecond()/otherMeasurment.toMetersPerSecond());
				}
	
	
	
}
