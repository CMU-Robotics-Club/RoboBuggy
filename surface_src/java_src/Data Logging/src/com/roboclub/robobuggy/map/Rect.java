package com.roboclub.robobuggy.map;
//TODO move to a polygon class 


public class Rect {
	private Point uR;
	private Point uL;
	private Point lR;
	private Point lL;
	
	
	/// ordering of points for Rect matter //TODO write requirement and assertions 
	public Rect(Point uR_, Point uL_, Point lR_, Point lL_) {
		this.uR = uR_;
		this.uL = uL_;
		this.lR = lR_;
		this.lL = lL_;
	}
	
	public boolean within(Point marker) {
		if (marker != null) {
			return (marker.getX() >= this.uL.getX()) && 
					(marker.getX() <= this.uR.getX()) &&
					(marker.getY() >= this.lL.getY()) && 
					(marker.getY() <= this.uL.getY());
		}
		
		return false;
	}
	
	@Override
	//TODO fix ordering of points matters 
	public boolean equals(Object obj){
		if(!(obj instanceof Rect)){
			return false;
		} 	
		Rect otherRect = (Rect)obj;
		//checks individual properties of the Rect for equality 
		if(!(uR.equals(otherRect.uR))){
			return false;
		}
		if(!(uL.equals(otherRect.uL))){
			return false;
		}
		if(!(lR.equals(otherRect.lR))){
			return false;
		}
		if(!(lL.equals(otherRect.lL))){
			return false;
		}
		//passed all equality requirements so the rects point our equivalent 
		return true;
	}
	
	
}
