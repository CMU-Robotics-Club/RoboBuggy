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
}
