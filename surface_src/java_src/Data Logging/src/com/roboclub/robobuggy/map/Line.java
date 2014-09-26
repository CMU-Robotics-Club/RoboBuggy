package com.roboclub.robobuggy.map;

public class Line {
	private Point start;
	private Point end;
	
	public Line(Point start,Point end){
		this.start = start;
		this.end = end;
	}
	
	//consider making template function right now is dependent on the type of point  
	public Line(double start_x,double start_y,double end_x,double end_y){
		Point startPoint = new Point((float)start_x,(float)start_y);
		Point endPoint = new Point((float)end_x,(float)end_y);
		this.start = startPoint;
		this.end = endPoint;
	}
	
	public Point getStart() {
		return start;
	}
	public void setStart(Point start) {
		this.start = start;
	}
	public Point getEnd() {
		return end;
	}
	public void setEnd(Point end) {
		this.end = end;
	}
	
	@Override
	public boolean equals(Object obj){
		if(!(obj instanceof Line)){
			return false;
		} 	
		Line otherLine = (Line)obj;
		
		//checks individual properties of the line for equality 
		if(!start.equals(otherLine.start)){
			return false;
		}
		if(!end.equals(otherLine.end)){
			return false;
		}
		//passed all equality requirements so two lines our equivlent 
		return true;
	}
	
	//TODO add almostEquals, display, collision, on ...
	//TODO add other if needed  properties ie color, width ....
}
