package com.roboclub.robobuggy.map;

public class Point {
	private float x;
	private float y;
	
	public Point(float start_x, float start_y) {
		this.x = start_x;
		this.y = start_y;
	}
	
	public float getX() {
		return this.x;
	}
	
	public float getY() {
		return this.y;
	}

	public void setX(float x_) {
		this.x = x_;
	}
	
	public void setY(float y_) {
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
