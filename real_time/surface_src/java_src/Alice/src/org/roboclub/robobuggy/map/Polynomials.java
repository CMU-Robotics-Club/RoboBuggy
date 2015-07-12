package org.roboclub.robobuggy.map;

import java.awt.Graphics;

import org.roboclub.robobuggy.coordinateFrame.FrameOfRefrence;
import org.roboclub.robobuggy.coordinateFrame.Pose;
import org.roboclub.robobuggy.linearAlgebra.Integer_Number;
import org.roboclub.robobuggy.linearAlgebra.Number;
import org.roboclub.robobuggy.linearAlgebra.Vector;
import org.roboclub.robobuggy.main.LogicException;



public class Polynomials extends MapObject{
	private Polynomial[] polys;
	
	/**
	 * TODO document
	 */
	Polynomials(FrameOfRefrence orgin,Polynomial[] polys){
		this.refrenceFrame = orgin.toPose();
		this.polys = polys;
	}
	
	@Override
	/**
	 * TODO document 
	 */
	boolean Equals(Object obj) {
		if(this.getClass() != obj){
			return false;
		}
		
		Polynomials otherPolynomial = (Polynomials)obj;
		
		//if one polynomial has a different number of nonzero coefficients 
		//then the other one  then the other one  must be a different polynomial
		if(polys.length != otherPolynomial.polys.length ){
			return false;
		}
		
		//could be speed up for big data sets with parallel operation 
		for (int i = 0;i<polys.length;i++) {
			//TODO make sure order of polys is the same 
			if(!polys[i].equals(otherPolynomial.polys[i])){
				return false;
			}
		}
		//has the same coefficients so must be the same polynomial
		return true;
	}

	@Override
	/**
	 * TODO document
	 * TODO implement
	 * @param obj
	 * @return
	 * @throws LogicException
	 */
	boolean isGreater(Object obj) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	/**
	 * TODO document
	 * TODO implement
	 * @param obj
	 * @return
	 * @throws LogicException
	 */
	boolean isLess(Object obj) throws LogicException {
		// TODO Auto-generated method stub
		return false;
	}
	
	@Override
	/**
	 * drawing function for the polynomial, will draw the object on the
	 *  jcomponent that the polynomial is added to
	 *  TODO implement 
	 */
	public void paintComponent(Graphics g){
		super.paintComponent(g);
		Vector x_unit = this.view.getXvector(); //the x vector of the image plane
		Vector y_unit = this.view.getYvector();//the y vector of the image plane
		
		//TODO make sure that polys is in the same order as x_unit and y_unit
		
		//project the polynomials to image plane
		Polynomial xPoly = Polynomial.zero();
		for(int i = 0;i<x_unit.getLength();i++){
			xPoly = xPoly.add(this.polys[i].mult(x_unit.getIndex(i)));
		}
		
		Polynomial yPoly = Polynomial.zero();
		for(int i = 0;i<x_unit.getLength();i++){
			yPoly = yPoly.add(this.polys[i].mult(y_unit.getIndex(i)));
		}		
		
		int nStart = -50;  //TODO set dynamically
		int nPoints = 100; //TODO set dynamically
		int[] xPoints = new int[nPoints];
		int[] yPoints = new int[nPoints];
		for(int n = nStart;n<nPoints-nStart;n++){
			try {
				xPoints[n] = xPoly.evaluateAt(new Integer_Number(n)).toInteger_Number().getValue();
				yPoints[n] = yPoly.evaluateAt(new Integer_Number(n)).toInteger_Number().getValue();
			} catch (LogicException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		g.drawPolyline(xPoints, yPoints, nPoints);
	}
}
