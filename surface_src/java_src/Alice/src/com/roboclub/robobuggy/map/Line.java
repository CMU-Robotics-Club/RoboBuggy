package com.roboclub.robobuggy.map;

import java.util.ArrayList;

public class Line implements MapObject{
	ArrayList<Point> points = new ArrayList<Point>();
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
		//TODO
		return false;
	}
	
	//returns true if the two lines intersect, false otherwise
	//TODO
	
	//returns the distance from a point to the line
	//TODO
	
	//combines linea and lineb into 1 continues line
	//TODO
	
	
	
	@Override
	public boolean equals(Object thisObject)
	{
		if(!(thisObject instanceof Line)){
			return false;
		}
		Line thisLine = (Line)thisObject;
		
		for(int i=0;i<points.size();i++)
		{
			if(!thisLine.onLine(points.get(i))){
				return false;
			}
		}
		
		for(int i=0;i<thisLine.points.size();i++){
			if(!onLine(thisLine.points.get(i))){
				return false;
			}
		}
		
		//every point in either line is on the other line so they must be equivlent lines
		return true;
	}

}
