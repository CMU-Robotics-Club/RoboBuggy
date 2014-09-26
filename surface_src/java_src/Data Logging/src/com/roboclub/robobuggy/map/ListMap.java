package com.roboclub.robobuggy.map;

import java.util.ArrayList;
import java.util.List;
//TODO figure out if locking is necessary, if need be add it 


public class ListMap implements Map{
	private List<MapObject> thisMap;
	
	public ListMap(){
		thisMap = new ArrayList<MapObject>();
	}
	
	public boolean removeFromMap(MapObject mapObjectToRemove){
		//TODO
		return true;
	}
	
	//if add is successful then evaluate to true, otherwise evaluate to false
	public boolean addToMap(MapObject newMapObject){
		thisMap.add(newMapObject);
		return true;
	}
	
	//if clear is successful then evaluate to true, otherwise evaluate to false 
	public boolean clearMap(){
		thisMap.clear();
		return true;
	}
	
	//evaluates a filtered copy of the map, does not effect the original map 
	public ListMap filterMap(){
		//TODO
		//hack
		return (ListMap) thisMap;
		
	}
	
	//TODO implement 
	
}
