package com.roboclub.robobuggy.map;

import java.util.ArrayList;

public class Line {
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

}
