package com.roboclub.robobuggy.map;

public class Point {
	private float x;
	private float y;
	
	public Point(float x_, float y_) {
		this.x = x_;
		this.y = y_;
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
}
