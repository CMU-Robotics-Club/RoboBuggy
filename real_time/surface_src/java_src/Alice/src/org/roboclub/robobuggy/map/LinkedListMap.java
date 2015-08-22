package org.roboclub.robobuggy.map;

import java.io.File;
import java.util.ArrayList;
import java.util.LinkedList;

import org.roboclub.robobuggy.coordinateFrame.FrameOfRefrence;
import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.measurments.Distince;
import org.roboclub.robobuggy.numbers.Number;
/***
 * 
 * @author Trevor Decker
 * @version 0.0
 * 
 * an implemention of a map that uses a linked list store the map nodes 
 */

public class LinkedListMap extends Map{
	LinkedList<MapObject> map;
	
	/***
	 * Constructor for the map which creates an empty map
	 */
	 public LinkedListMap() {
		 map = new LinkedList<MapObject>();
	 }
	
	@Override
	/***
	 * evaluates to the number of elements currently on the map 
	 */
	public int getNumberOfElements(){
		return map.size();
	}

	
	@Override
	/***
	 * evaluates to the single object which is deemed closest to a given point
	 * if multiple objects are equally close only one of them will be returned
	 * if no closestObject exists ie the map is empty return null
	 * @param aPoint
	 * @return
	 * @throws LogicException 	
	 */
	public MapObject getClosestObject(FrameOfRefrence aPoint) throws LogicException, CloneNotSupportedException {
		if(map.size() <1){
			return null;
		}
		
		int best_index = 0;
		Distince best_distince = map.get(0).getMinDistince(aPoint);
		for(int i = 1;i<map.size();i++){
			Distince this_distince = map.get(i).getMinDistince(aPoint);
			if(this_distince.isLess(best_distince)){
				best_index = i;
				best_distince = this_distince;
			}
		}
		return map.get(best_index);
	}



	@Override
	/***
	 * TODO document
	 * TODO implement
	 */
	public ArrayList<MapObject> getPointsInRange(ArrayList<Point> Points) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/***
	 * TODO document
	 * TODO implement
	 */
	public ArrayList<MapObject> getPointsOutSideRange(ArrayList<Point> Points) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/***
	 * TODO document
	 * TODO implement
	 */
	public ArrayList<MapObject> getPointsOnRange(ArrayList<Point> Points) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	/***
	 * TODO document
	 */
	public boolean isObjectOnMap(MapObject anObject) {
		for(int i = 0;i<map.size();i++){
			if(anObject.Equals(map.get(i))){
				return true;
			}
		}		
		return false;
	}

	@Override
	/***
	 * Adds an object to the map
	 * Work(1)
	 * Span(1)
	 */
	public void AddObject(MapObject anObject) {
		map.add(anObject);		
	}

	@Override
	/***
	 * removes all objects that are equivalent to anObject
	 * Work(sizeOfMap)
	 * Span(sizeOfMap)
	 */
	public void removeObject(MapObject anObject) {		
		for(int i = 0;i< map.size();i++){
			if(map.get(i).equals(anObject)){
				map.remove(i);
			}
			i--; //To make up for the fact that we removed an object 
		}		
	}

	@Override
	/***
	 * TODO implment
	 * TODO documnet
	 */
	public void loadMap(File mapData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	/***
	 * TODO implment
	 * TODO documnet 
	 */
	public void saveMap(File mapData) {
		// TODO Auto-generated method stub
		
	}

	@Override
	/**
	 * TODO document
	 */
	public MapObject[] getClosestNObjects(FrameOfRefrence aFrame,
			int numPoints) throws LogicException, CloneNotSupportedException {
		//a sorted array of mapObjects (sorted based on distance from aPoint)
				MapObject[] sortedObjects = new MapObject[numPoints];
				//sets all of the points to null incase not enogth objects exsits on the map we do not want 0 objects to be returned
				for(int i = 0;i<numPoints;i++){
					sortedObjects[i] = null;
				}
				//TODO make a binary search 
				for(int i = 0;i<map.size();i++)
				{
					//add this element to the sortedObjects list (if it is within range)
					int offset = 0;
					Number mapObjectDistince = (Number) map.get(i).getMinDistince(aFrame).toMeters().getMeassurmentValue(); 
					while(offset < numPoints && sortedObjects[offset] != null && sortedObjects[offset].getMinDistince(aFrame).isLess(mapObjectDistince))
					{
						offset++;
					}
					//we have reached the point in the list where the new element should be added 
					//shift all points after this one 
					for(int j = numPoints-1;j> offset;j--)
					{
						sortedObjects[j] = sortedObjects[j-1];
					}
					//add this point 
					if(offset < numPoints){
					sortedObjects[offset] = map.get(i);
					}
					
				}
				return sortedObjects;
	}

	

	
}
