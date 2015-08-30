package org.roboclub.robobuggy.measurements;

import javax.print.attribute.standard.MediaSize.Other;

public class Volume extends Measurement{
	//the internal unit for volume is meters^3
	private Volume(double value){
		this.value = value;
	}
	
	public static Volume metersCube(double number){
		return new Volume(number);
	}
	
	public double toMetersCube(){
		return value;
	}
	
	public String toString(){
		return this.toMetersCube() + " meters^3";
	}
	
	public Volume add(Volume otherVolume){
		return  new Volume(this.value + otherVolume.value);
	}
	
	public Volume subtract(Volume otherVolume){
		return new Volume(this.value - otherVolume.value);
	}
	
	public Volume multiply(double scale){
		return new Volume(this.value*scale);
		}
	
	public Volume divide(double scale){
		return new Volume(this.value/scale);
		}
	
	public Area divide(Distince someDistince){
		return Area.metersSq(this.toMetersCube()/someDistince.toMeters());
	}
	
	public Distince divide(Area someArea){
		return Distince.meter(this.toMetersCube()/someArea.toMetersSq());
	}
	
	public VolumePerTime divide(Time someTime){
		return VolumePerTime.meterCubePerSecond(this.toMetersCube()/someTime.toSeconds());
	}
	
}
