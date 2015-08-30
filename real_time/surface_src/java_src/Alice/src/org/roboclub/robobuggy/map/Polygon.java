package org.roboclub.robobuggy.map;

import java.awt.Color;
import java.awt.Graphics;

import org.roboclub.robobuggy.main.LogicException;

//TODO

public class Polygon extends MapObject{
	private Point[] vertices;
	private Color backgroundColor;
	private Color outlineColor;

	//TODO add constructor 
	//TODO add adder/deleter
	//TODO add getter setters 
	//TODO add image stuff 
	
	@Override
	boolean Equals(Object obj) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	boolean isGreater(Object obj) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	boolean isLess(Object obj) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	/**
	 * drawing function for the Point, will draw the object on the
	 *  jcomponent that the Point is added to
	 *  TODO implement 
	 */
	public void paintComponent(Graphics g){
		super.paintComponent(g);
		int[] xPoints = new int[vertices.length];
		int[] yPoints = new int[vertices.length];
		for(int i = 0;i<vertices.length;i++){
			xPoints[i] = vertices[i].getX();
			yPoints[i] = vertices[i].getY();
		}
        g.setColor(backgroundColor);
		g.drawPolygon(xPoints, yPoints, vertices.length);
        g.setColor(outlineColor);
		g.fillPolygon(xPoints, yPoints, vertices.length);

	}
	
	
}