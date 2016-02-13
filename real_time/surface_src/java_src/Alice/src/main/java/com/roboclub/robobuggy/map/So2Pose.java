package com.roboclub.robobuggy.map;

import Jama.Matrix;

/**
 * 
 * @author Trevor Decker
 * Representation of a so2 point (x,y,orientation) 
 *
 */
public class So2Pose {
	private Point location;
	private double orintation; 
	
	/**
	 * 
	 * @param newLocation
	 * @param newOrintation
	 */
	So2Pose(Point newLocation,double newOrintation){
		this.location = location;
		this.orintation = newOrintation;
	}
	
	/**
	 * 
	 * @param x
	 * @param y
	 * @param newOrintation
	 */
	public So2Pose(double x,double y,double newOrintation){
		location = new Point(x, y);
		orintation = newOrintation;
	}
	
	/**
	 * 
	 * @param postPose the pose that is being applied to the right of the expresion
	 */
	public So2Pose mult(So2Pose postPose){
		double[][] aM = {{Math.cos(orintation), -Math.sin(orintation), getX()},
				         {Math.sin(orintation), Math.cos(orintation),getY()},
				         {0,0,1}};
		double[][] bM = {{Math.cos(postPose.orintation), -Math.sin(postPose.orintation), postPose.getX()},
		         		{Math.sin(postPose.orintation), Math.cos(postPose.orintation),postPose.getY()},
		         		{0,0,1}};
		Matrix a = new Matrix(aM);
		Matrix b = new Matrix(bM);
		Matrix c = a.times(b);

		
		return new So2Pose(c.get(0, 2), c.get(1,2), Math.atan2(c.get(1, 0), c.get(0, 0)));
		
	}
	
	
	/*
	 * updates the values of the pose
	 * @param Point newPoint the new se2 point to be set
	 * @param double newOrintation the new orientation 
	 */
	public void updatePoint(Point newPoint,double newOrintation){
		this.orintation = orintation;
		this.location = location;

	}
	
	/**
	 * returns the most recent position (se2 point) value 
	 * @return se2 Point
	 */
	public Point getSe2Point(){
		return location;
	}
	
	/**
	 * 
	 * @return the x coordinate of the of the se2 position of the pose 
	 */
	public double getX(){
			return location.getX();
	}
	
	/**
	 * 
	 * @return the y coordinate of the se2 position of the pose 
	 */
	public double getY(){
			return location.getY();
	}
	
	/**
	 * 
	 * @return the orientation of the pose
	 */
	public double getOrintation(){
		return orintation;
	}

}
