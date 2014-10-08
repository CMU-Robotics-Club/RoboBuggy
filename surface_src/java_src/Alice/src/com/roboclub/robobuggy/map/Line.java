package com.roboclub.robobuggy.map;

import java.util.ArrayList;

public class Line implements MapObject{
	ArrayList<Point> points = new ArrayList<Point>();
	final static double ON_LINE_DISTANCE = .1; //meters
	//must have at least two points

	Line(ArrayList<Point> newPoints){
		for(int i = 0;i<newPoints.size();i++){
			points.add(newPoints.get(i));
		}
	}

	public void addPointToLine(Point newPoint){
		points.add(newPoint);
	}
	
	public void removePoint_byIndex(int i){
		points.remove(i);
	}
	
	public void removeEquivlentPoint(Point pointToRemove){
		for(int i =0;i<points.size();i++){
			if(points.get(i).equals(pointToRemove)){
				points.remove(i);
				i--;  //decrement point since we removed it
			}
		}
	}
	
	//returns true if aPoint is on the line, flase otherwise 
	public boolean onLine(Point aPoint){
		return getDistance(Point aPoint) < ON_LINE_DISTANCE;
	}
	
	//returns true if the two lines intersect, false otherwise
	public boolean intersect(Line aLine){
		// TODO
	}
	
	//returns the distance from a point to the line
	// returns in meters 
	public double getDistance(Point aPoint){
		Point closestPoint = projectPointToLine(aPoint);
		return aPoint.getDistance(closestPoint);
	}

	public Point projectPointToLine(Point aPoint){
		//TODO
	}
	
	//combines line_a and line_b into 1 continuous line
	public Line combineLine(Line line_a,Line line_b){
		//TODO
	}

	/// returns true if aLine is a subsection of the line
	public boolean isSubSection(Line thisLine){
		for(int i=0;i<thisLine.points.size();i++)
		{
			if(!onLine(thisLine.points.get(i))){
				return false;
			}
		}
		return true;
	}

	
	
	@Override
	public boolean equals(Object thisObject)
	{
		if(!(thisObject instanceof Line)){
			return false;
		}
		Line thisLine = (Line)thisObject;

		if(!thisLine.isSubSection(this)){
			return false;
		}

		if(isSubSection(thisLine)){
			return false;
		}
	
		// every point on either line is on the other line so they
		// must be equivlent lines
		return true;
	}

}
