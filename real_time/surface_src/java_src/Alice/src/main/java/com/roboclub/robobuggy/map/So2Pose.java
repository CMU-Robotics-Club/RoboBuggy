package com.roboclub.robobuggy.map;

import com.roboclub.robobuggy.main.Util;

import Jama.Matrix;

/**
 * 
 * @author Trevor Decker
 * Representation of a so2 point (x,y,orientation) 
 *
 */
public class So2Pose {
	private Point location;
	private double orientation; //in radians

	/**
	 * 
	 * @param newLocation the new location
	 * @param newOrientation the new orientation
	 */
	public So2Pose(Point newLocation,double newOrientation){
		this.location = newLocation;
		this.orientation = Util.normilizeAngleRad(newOrientation);
	}
	
	/**
	 * 
	 * @param x x cord of the point
	 * @param y y cord of the point
	 * @param newOrientation the new orientation
	 */
	public So2Pose(double x,double y,double newOrientation){
		location = new Point(x, y);
		orientation = Util.normilizeAngleRad(newOrientation);
	}
	
	/**
	 * 
	 * @param postPose the pose that is being applied to the right of the expression
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
		double th = Util.normilizeAngleRad(Math.atan2(c.get(1, 0), c.get(0, 0)));
		return new So2Pose(c.get(0, 2), c.get(1,2), th);
		
	}
	
	/**
	 * evaluates to the inverse of the so2 pose (the position change needed to get to zero) 
	 * @return an So2Pose object that is the inverse of the current object
	 */
	public So2Pose inverse(){
		double[][] mArray = {{Math.cos(orientation),-Math.sin(orientation),getX()},
						{Math.sin(orientation),Math.cos(orientation),getY()},
						{0,0,1}};
		Matrix mMatrix = new Matrix(mArray);
		Matrix mMatrixInv = mMatrix.inverse();
		double th = Util.normilizeAngleRad(Math.atan2(mMatrixInv.get(1, 0),mMatrixInv.get(0, 0)));
		return new So2Pose(mMatrixInv.get(0, 2),mMatrixInv.get(1,2),th);
	}
	
	
	/**
	 * updates the values of the pose
	 * @param newPoint the new se2 point to be set
	 * @param newOrientation the new orientation
	 */
	public void updatePoint(Point newPoint, double newOrientation){
		this.orientation = Util.normilizeAngleRad(newOrientation);
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
	public static So2Pose identity(){
		return new So2Pose(0.0, 0.0,0.0);
	}

	/**
	 * hashcode function needed for storeing the object
	 */
	@Override
	public int hashCode()
	{
		return (int)(getOrientation()*getX()*1000);
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
	 * evaluates to a string encoding information about this class
	 * @return a string encoding what this objects information 
	 */
	public String toString(){
		return "{So2Pose | x: "+getX()+", y: "+getY()+", orintation:"+getOrientation()+"}";
		
	}
}
