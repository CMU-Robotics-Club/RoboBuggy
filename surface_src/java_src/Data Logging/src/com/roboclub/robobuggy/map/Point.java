package com.roboclub.robobuggy.map;

public class Point {
	private double x;
	private double y;
	
	public Point(double x_, double y_) {
		this.x = x_;
		this.y = y_;
	}
	
	public double getX() {
		return this.x;
	}
	
	public double getY() {
		return this.y;
	}

	public void setX(double x_) {
		this.x = x_;
	}
	
	public void setY(double y_) {
		this.y = y_;
	}
	
	@Override
	public boolean equals(Object obj){
		if(!(obj instanceof Line)){
			return false;
		} 	
		Point otherPoint = (Point)obj;
		//checks individual properties of the line for equality 
		if(!(x == otherPoint.x)){
			return false;
		}
		if(!(y == otherPoint.y)){
			return false;
		}
		//passed all equality requirements so two point our equivalent 
		return true;
	}
}
