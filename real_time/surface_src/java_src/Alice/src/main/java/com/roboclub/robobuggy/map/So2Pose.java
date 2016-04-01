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
	private double orientation;

	/**
	 * 
	 * @param newLocation the new location
	 * @param newOrientation the new orientation
	 */
	public So2Pose(Point newLocation,double newOrientation){
		this.location = newLocation;
		this.orientation = newOrientation;
	}
	
	/**
	 * 
	 * @param x x coord of the point
	 * @param y y coord of the point
	 * @param newOrientation the new orientation
	 */
	public So2Pose(double x,double y,double newOrientation){
		location = new Point(x, y);
		orientation = newOrientation;
	}
	
	/**
	 * 
	 * @param postPose the pose that is being applied to the right of the expresion
	 * @return the new So2Pose TODO
	 */
	public So2Pose mult(So2Pose postPose){
		double[][] aM = {{Math.cos(orientation), -Math.sin(orientation), getX()},
				         {Math.sin(orientation), Math.cos(orientation),getY()},
				         {0,0,1}};
		double[][] bM = {{Math.cos(postPose.orientation), -Math.sin(postPose.orientation), postPose.getX()},
		         		{Math.sin(postPose.orientation), Math.cos(postPose.orientation),postPose.getY()},
		         		{0,0,1}};
		Matrix a = new Matrix(aM);
		Matrix b = new Matrix(bM);
		Matrix c = a.times(b);

		
		return new So2Pose(c.get(0, 2), c.get(1,2), Math.atan2(c.get(1, 0), c.get(0, 0)));
		
	}
	
	/**
	 * evaluates to the inverse of the so2 pose (the position change needed to get to zero) 
	 * @return an So2Pose object that is the inverse of the current object
	 */
	public So2Pose inverse(){
		double[][] m = {{Math.cos(orientation),-Math.sin(orientation),getX()},
						{Math.sin(orientation),Math.cos(orientation),getY()},
						{0,0,1}};
		Matrix M = new Matrix(m);
		Matrix M_inv = M.inverse();
		double th = Math.atan2(M_inv.get(1, 0),M_inv.get(0, 0));
		return new So2Pose(M_inv.get(0, 2),M_inv.get(1,2),th);
	}
	
	
	/**
	 * updates the values of the pose
	 * @param newPoint the new se2 point to be set
	 * @param newOrientation the new orientation
	 */
	public void updatePoint(Point newPoint, double newOrientation){
		this.orientation = newOrientation;
		this.location = newPoint;

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
	public double getOrientation(){
		return orientation;
	}
	
	/**
	 * Evaluates to the identity object for So2Pose (no position, or orientation change) 
	 * @return the Identity So2Pose
	 */
	public static So2Pose Identity(){
		return new So2Pose(0.0, 0.0,0.0);
	}
	
	/**
	 * equals function for So2Pose that can be used to check if two psoes are the same 
	 * @return equality 
	 */
	@Override
	public boolean equals(Object o){
		if(!(o instanceof So2Pose)){
			return false;
		}
		
		So2Pose otherPose = (So2Pose)o;
		if(Math.abs(otherPose.getX() - getX()) > .0001){
			return false;
		}
		
		if(Math.abs(otherPose.getY() - getY()) > .0001){
			return false;
		}
		
		if(Math.abs(otherPose.getOrientation() - getOrientation()) > .0001){
			return false;
		}
		
		//all values were equal so the two poses represent the same pose aka they are equal 
		return true;
	}

	
	/**
	 * evaluates to a string encoding infromation about this class
	 */
	public String toString(){
		return "{So2Pose | x: "+getX()+", y: "+getY()+", orintation:"+getOrientation()+"}";
		
	}
}
