package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.linearAlgebra.Matrix;
import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.measurments.Distince;
import org.roboclub.robobuggy.numbers.Double_Number;
import org.roboclub.robobuggy.numbers.Number;

/**
 * TODO document 
 * @author Trevor Decker
 * @version 0.0
 *
 */
public class HomogeneousMatrix implements FrameOfRefrence {
	protected int dimension; //size of the matrix should be set by each implementation 
	protected Matrix<Double_Number> thisMatrix;
	//encodes the order of the directions of the system 
	protected SpacialDimensions[] directions;
	
	
	/**
	 * TODO document
	 * TODO implement
	 */
	public HomogeneousMatrix() {
		// TODO Auto-generated constructor stub
	}
	
	@Override
	/**
	 * TODO document 
	 */
	public FrameOfRefrence preApply(FrameOfRefrence otherFrame) throws LogicException {
		//TODO make no side effects
		thisMatrix = otherFrame.toHomogeneousMatrix().thisMatrix.mult(thisMatrix);
		return this;
	}

	@Override
	/**
	 * TODO document 
	 */
	public FrameOfRefrence postApply(FrameOfRefrence otherFrame) throws LogicException {
		//TODO make no side effects
		thisMatrix = thisMatrix.mult(otherFrame.toHomogeneousMatrix().thisMatrix);
		return this;
	}

	@Override
	/**
	 * TODO document 
	 */
	public FrameOfRefrence inverse() throws LogicException {
		//TODO remvoe try catch 
		HomogeneousMatrix result = null;
		try {
			 result = (HomogeneousMatrix) this.clone();
		} catch (CloneNotSupportedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		result.thisMatrix = thisMatrix.inverse();
		return result;
	}
	
	/**
	 * TODO document 
	 * TODO implement
	 * @param newRotation
	 */
	public void setRotatonalRepersentation(RotationalRepersentation newRotation){
			//TODO implment
	}




	
	/**
	 * TODO document 
	 */
	public static <NUMTYPE extends Number>FrameOfRefrence zero() {
		//TODO make so that matrix has rotation and distince encoded not just doubles
		Point zP = Point.zero();
		RotationMatrix<NUMTYPE> zR = (RotationMatrix<NUMTYPE>) RotationMatrix.zero();
		HomogeneousMatrix newFrame = new HomogeneousMatrix();
		return newFrame;
	}
	
	@Override
	/**
	 * TODO document
	 */
	public HomogeneousMatrix toHomogeneousMatrix() {
		return this;
	}
	
	
	
	/**
	 * TODO document
	 */
	@Override
	public PositionRepersentation getPosition() {
		return new Point(this.thisMatrix.getCol(dimension-1)); //TODO remove the last row 
	}
	
	@Override
	/**
	 * TODO document
	 */
	public RotationalRepersentation getOrintation() {
		return new RotationMatrix<Double_Number>(thisMatrix.getCols(0,dimension-1).getRows(0, dimension-1));
	}

	@Override
	/**
	 * TODO document
	 * TODO implement
	 */
	public Pose toPose() {
		// TODO Auto-generated method stub
		return null;
	}
	
}
