package com.roboclub.robobuggy.map;

import java.util.List;

/**
 * Object used to represent a map of the course
 */
public class Map implements MapObject{
	
	/**
	 * Returns the closest {@link MapBoject} object to a {@link Point} in the 
	 * {@link Map}
	 * @param aPoint {@link Point} to look near
	 * @return the closest {@link MapBoject} object to a {@link Point} in the
	 * {@link Map}
	 */
	public MapObject getClosestObject(Point aPoint){
		//TODO
		return null;
	}
	
	/**
	 * Returns the closest n {@link MapObject} objects to a {@link Point}
	 *  in the {@link Map}
	 * @param aPoint {@link Point} to look near
	 * @param n number of {@link Mapobject}s to return
	 * @return  the closest n {@link MapObject} objects to a {@link Point}
	 *  in the {@link Map}
	 */
	public List<MapObject> getClosestNObjects(Point aPoint, int n){
		//TODO
		return null;
	}
	
	// @REQUIER num(Points) > 2
	/**
	 * TODO: Figure out what this function does
	 * @param points {@link List} of {@link Point}s
	 * @return TODO: Figure out what is returned
	 */
	public List<MapObject> getPointsInRange(List<Point> points){
		//TODO
		return null;
	}
	
	//@REQUIER num(Points) >2
	/**
	 * TODO: Figure out what this function does
	 * @param points {@link List} of {@link Point}s
	 * @return TODO: Figure out what is returned
	 */
	public List<MapObject> getPointsOutSideRange(List<Point> points){
		//TODO
		return null;
	}
	
	//@REQUIER num(Points) > 2
	/**
	 * TODO: Figure out what this function does
	 * @param points {@link List} of {@link Point}s
	 * @return TODO: Figure out what is returned
	 */
	public List<MapObject> getPointsOnRange(List<Point> points){
		//TODO
		return null;
	}
	
	/**
	 * Determines if a {@link MapObject} is contained within the {@link Map}
	 * @param anObject {@link MapObject} to check
	 * @return true iff anObject is within the {@link Map}
	 */
	public boolean isObjectOnMap(MapObject anObject){
		//TODO
		return false;
	}
	
	/**
	 * Adds a {@link MapObject} to the {@link Map}
	 * @param anObject {@link MapObject} to add
	 */
	public void addObject(MapObject anObject){
		//TODO
	}
	
	/**
	 * Removes a {@link MapObject} to the {@link Map}
	 * @param anObject {@link MapObject} to remove
	 */
	public void removeObject(MapObject anObject){
		//TODO
	}
	
}
