package org.roboclub.robobuggy.map;

import java.io.File;
import java.util.ArrayList;

import org.roboclub.robobuggy.coordinateFrame.FrameOfRefrence;
import org.roboclub.robobuggy.main.LogicException;

/***
 * 
 * @author Trevor Decker
 * TODO document
 * @version 0.0
 * 
 *	TODO
 */
public abstract class Map {

	/***
	 * evaluates to the number of elements currently on the map 
	 * 
	 */
	public abstract int getNumberOfElements();
	
	 // Representation of where the maps coordinates are defined from
	FrameOfRefrence refrenceFrame;
	
	/***
	 * TODO document
	 * @param aPoint
	 * @return
	 * @throws LogicException 
	 */
	public abstract MapObject getClosestObject(FrameOfRefrence aPoint) throws LogicException;
	
	/***
	 * TODO document
	 * @param pointOfView
	 * @return
	 * @throws LogicException 
	 */
	public abstract MapObject[] getClosestNObjects(FrameOfRefrence pointOfView,int numPoints) throws LogicException;
	
	// @REQUIER num(Points) > 2
	/*** 
	 * TODO document
	 * @param Points
	 * @return
	 */
	public abstract ArrayList<MapObject> getPointsInRange(ArrayList<Point> Points);
	
	//@REQUIER num(Points) >2
	/***
	 * TODO document
	 * @param Points
	 * @return
	 */
	public abstract ArrayList<MapObject> getPointsOutSideRange(ArrayList<Point> Points);
	
	//@REQUIER num(Points) > 2
	/***
	 * TODO documnet
	 * @param Points
	 * @return
	 */
	public abstract ArrayList<MapObject> getPointsOnRange(ArrayList<Point> Points);
	/***
	 * TODO document
	 * @param anObject
	 * @return
	 */
	public abstract boolean isObjectOnMap(MapObject anObject);
	
	//Add Objects 
	/***
	 * TODO document
	 * @param anObject
	 */
	public abstract void AddObject(MapObject anObject);
	/***
	 * TODO document
	 * @param anObject
	 */
	public abstract void removeObject(MapObject anObject);
	
	/***
	 * TODO document 
	 * @param mapData
	 */
	public abstract void loadMap(File mapData);
	
	/***
	 * TODO document
	 * @param mapData
	 */
	public abstract void saveMap(File mapData);
	
	
}
