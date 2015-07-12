package org.roboclub.robobuggy.map;

import java.awt.Component;
import java.awt.Graphics;

import javax.swing.JComponent;

import org.roboclub.robobuggy.coordinateFrame.FrameOfRefrence;
import org.roboclub.robobuggy.coordinateFrame.Pose;
import org.roboclub.robobuggy.coordinateFrame.SpacialDimensions;
import org.roboclub.robobuggy.linearAlgebra.Distince;
import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.Order;

/***
 * 
 * @author Trevor Decker
 * @version 0.0
 *
 */
public abstract class  MapObject extends  JComponent{
	
	
	 //TODO document
	Pose refrenceFrame;
	protected View view;  //how the map object should be viewed 

	
	/**
	 * TODO document
	 */
	abstract boolean Equals(Object obj);
	abstract boolean isGreater(Object obj) throws LogicException;
	abstract boolean isLess(Object obj) throws LogicException;
	
	/***
	 * TODO document 
	 * @param obj
	 * @return
	 * @throws LogicException 
	 */
	Order compare(Object obj) throws LogicException{
		//if the two objects are of diffrent classes then it does not make sense to compare them 
		if(obj.getClass() != this.getClass()){
			return Order.NA;
		}
		if(Equals(obj)){
			return Order.EQUAL;
		}else if(isGreater(obj)){
			return Order.GREATER;
		}else{
			return Order.LESS;
		}
		
	}
	
	/**
	 * evaluates to the transform between this points frame of reference to aPoints reference Frame
	 * @param obj
	 * @return
	 * @throws LogicException 
	 */
	public FrameOfRefrence getTransform(MapObject otherObject) throws LogicException {
		return this.refrenceFrame.postApply(otherObject.refrenceFrame.inverse());
	}
	
	/**
	 * TODO document
	 * @param aPoint
	 * @return
	 * @throws LogicException 
	 */
	public Distince getMinDistince(FrameOfRefrence aPoint) throws LogicException{
		return  getTransform((MapObject) aPoint).getPosition().getDistince();
	}
}
