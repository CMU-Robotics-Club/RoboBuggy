package org.roboclub.robobuggy.map;

import java.awt.Graphics;
import java.awt.Graphics2D;

import javax.swing.JFrame;

import org.roboclub.robobuggy.coordinateFrame.FrameOfRefrence;
import org.roboclub.robobuggy.coordinateFrame.SpacialDimensions;
import org.roboclub.robobuggy.linearAlgebra.Matrix;
import org.roboclub.robobuggy.linearAlgebra.Vector;
import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.measurments.Distince;
import org.roboclub.robobuggy.numbers.Double_Number;
import org.roboclub.robobuggy.numbers.Integer_Number;

/**
 * 
 * @author Trevor Decker
 * A view is a perspective and scale and frame to draw objects onto. 
 * It is the structure for storing all of the information about how
 * a scene looks
 */
public class View extends JFrame{
	// where the "camera" for the view should be located, 
	// this is the prospective used for the view
	FrameOfRefrence pointOfVeiw;
	//TODO document
	Distince[] scale;
	//TODO document
	SpacialDimensions[] directions;
	
	/**
	 * TODO document
	 * TODO implement
	 */
	public Vector getXvector(){
		return null;
	}
	
	/**
	 * TODO document
	 * TODO implement
	 * @return
	 */
	public Vector getYvector(){
		return null;
	}
	
	public void paint (Graphics g) {
	    Graphics2D g2 = (Graphics2D) g;
	    g2.clearRect(0, 0, this.getWidth(), this.getHeight());
	    try {
			DrawAxis(g2);
		} catch (LogicException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * TODO document
	 * TODO implement
	 * @param cooerdimates
	 * @return
	 */
	private int[] projectPointToView_X(int[] cooerdimates){
		return cooerdimates;
		//TODO
	}
	
	/**
	 * TODO document
	 * TODO implement
	 * @throws LogicException 
	 */
	public void DrawAxis(Graphics2D g2) throws LogicException{
		//Vector<Integer_Number> axis1 = new Vector<Integer_Number>(sample,3);
		Vector<Integer_Number> xaxis = new Vector(new Integer_Number(1),new Integer_Number(0),new Integer_Number(0));
		Integer_Number a = new Integer_Number(1);
		Integer_Number b = new Integer_Number(-1);
		Matrix<Integer_Number> xaxis_a = xaxis.scale(a); //TODO fix so that we can cast to a vector instead of a matrix
		Matrix<Integer_Number> xaxis_b = (Vector<Integer_Number>) xaxis.scale(b);
		Vector<Integer_Number> test = xaxis.fromMatrix(xaxis_a);
		
		// project to view
		Vector<Integer_Number> xView = new Vector<Integer_Number>(new Integer_Number(1), new Integer_Number(0),new Integer_Number(0));
		Vector<Integer_Number> yView = new Vector<Integer_Number>(new Integer_Number(0), new Integer_Number(1),new Integer_Number(0));
/*		Integer_Number x_num_a = (Integer_Number) xView.dotProduct(xaxis_a, Integer_Number.zero());
		Integer_Number y_num_a = (Integer_Number) yView.dotProduct(xaxis_a, Integer_Number.zero());
		Integer_Number x_num_b = (Integer_Number) xView.dotProduct(xaxis_b, Integer_Number.zero());
		Integer_Number y_num_b = (Integer_Number) yView.dotProduct(xaxis_b, Integer_Number.zero()); 
		//g2.drawLine(x_num_a.getValue(),y_num_a.getValue(), x_num_b.getValue(), y_num_b.getValue());
		
//		System.out.println(xView.transpose());
		System.out.println("end");
*/
		//TODO project to y_view
		
		/*
		 * 		int axis1[] = {1,0,0};
		
		int xaxis[] = {1, 0, 0};
		int yaxis[] = {0, 1, 0};
		
		int a = 1;
		int b = -1;
		int axis1_a[] = {a*axis1[0],a*axis1[1],a*axis1[2]};
		int axis1_b[] = {b*axis1[0],b*axis1[1],b*axis1[2]};
		//TODO get projected line, find intersection of that line with permiter of frame
		// plot the line connecting those 2 points
		// do not plot if the line does not intersect the permiter
		projectPointToView_X(xaxis);
		*/
		int dy = 10;
		int dz = 1;
		
		
		//TODO make axis based on view not just hard coded
		//axis 1
		int x1 = this.getWidth()/2;
		int x2 = this.getWidth()/2;
		int y1 = 0;
		int y2 = this.getHeight();
		g2.drawLine(x1, y1, x2, y2);
		//axis 2
		x1 = 0;
		x2 = this.getWidth();
		y1 = this.getHeight()/2;;
		y2 = this.getHeight()/2;
		g2.drawLine(x1, y1, x2, y2);
		
		
		//TOOD
	}

}
