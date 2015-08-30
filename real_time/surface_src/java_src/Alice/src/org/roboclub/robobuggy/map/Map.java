package org.roboclub.robobuggy.map;

import java.io.File;
import java.util.ArrayList;

import org.roboclub.robobuggy.coordinateFrame.FrameOfRefrence;
import org.roboclub.robobuggy.main.LogicException;

/***
 * 
 * @author Trevor Decker
 * definition of what a map is and can do, enables for users of maps
 * to use the maps functions abstractly.  This will allow for different 
 * kinds of maps to be used for different applications.
 * @version 0.0
 * 
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
	 * evaluates to the single object which is deemed closest to a given point
	 * if multiple objects are equally close only one of them will be returned
	 * if no closestObject exists ie the map is empty return null
	 * @param aPoint
	 * @return
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public abstract MapObject getClosestObject(FrameOfRefrence aPoint) throws LogicException, CloneNotSupportedException;
	
	/***
	 * evaluates to the n closest object to a given point
	 * if multiple objects are equally close a subset of them will be returned
	 * @param pointOfView
	 * @return
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public abstract MapObject[] getClosestNObjects(FrameOfRefrence pointOfView,int numPoints) throws LogicException, CloneNotSupportedException;
	
	/*** 
	 * TODO document
	 * @param Points
	 * @REQUIER num(Points) > 2
	 * @return
	 */
	public abstract ArrayList<MapObject> getPointsInRange(ArrayList<Point> Points);
	
	/***
	 * TODO document
	 * @param Points
	 * @REQUIER num(Points) >2
	 * @return
	 */
	public abstract ArrayList<MapObject> getPointsOutSideRange(ArrayList<Point> Points);
	
	/***
	 * TODO documnet
	 * @param Points
	 * @REQUIER num(Points) > 2
	 * @return
	 */
	public abstract ArrayList<MapObject> getPointsOnRange(ArrayList<Point> Points);

	/***
	 * Evaluates to true if the object in question (anObject) is equal to some object
	 * on this map, false otherwise
	 * @param anObject
	 * @return
	 */
	public abstract boolean isObjectOnMap(MapObject anObject);
	
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
	 * loads a map from a file
	 * @param mapData
	 */
	 public abstract  void loadMap(File mapData);
	
	/***
	 * saves the current map to file which can be loaded as a map later
	 * @param mapData
	 */
	public abstract void saveMap(File mapData);
	
	
}
