package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.linearAlgebra.Matrix;
import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.numbers.Double_Number;
import org.roboclub.robobuggy.numbers.Number;

/**
 * 
 * @author Trevor Decker
 * @version 0.0
 * @param <TYPE>
 * TODO documnet 
 */
public class RotationMatrix <TYPE extends Number>implements RotationalRepersentation{
	private Matrix<TYPE> thisMatrix;
	
	//Constructor
	/**
	 * TODO document 
	 * @param newMatrix
	 */
	public  RotationMatrix(Matrix<TYPE> newMatrix) {
		thisMatrix = newMatrix;
	}
	  
	@Override
	/**
	 * TODO document
	 * NOTE assuming that order of coordinate frames is in the same order TODO keep track of this in future releases 
	 */
	public RotationalRepersentation preApplyRotation(RotationalRepersentation secondRotation) throws LogicException {
		//TODO remove try catch 
		RotationMatrix<TYPE> result = null;
		try {
			result = (RotationMatrix<TYPE>) this.clone();
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		result.thisMatrix = secondRotation.toRotationMatrix().thisMatrix.mult(thisMatrix);
		return result;
	}

	@Override
	/**
	 * TODO document 
	 *  NOTE assuming that order of coordinate frames is in the same order TODO keep track of this in future releases 
	 */
	public RotationalRepersentation postApplyRotation(RotationalRepersentation secondRotation) throws LogicException {
		//TODO remove try catch 
		RotationMatrix<TYPE> result = null;
		try {
			result = (RotationMatrix<TYPE>) this.clone();
		} catch (CloneNotSupportedException e) {
			e.printStackTrace();
		}
		result.thisMatrix = thisMatrix.mult(secondRotation.toRotationMatrix().thisMatrix);
		return result;
	}

	@Override
	/**
	 * TODO document
	 */
	public RotationMatrix toRotationMatrix() {
		return this;
	}

	@Override
	/**
	 * TODO document 
	 * TODO implement
	 */
	public EulerAngles toEulerAngles() {
/*
		//http://staff.city.ac.uk/~sbbh653/publications/euler.pdf
//		double angle = Math.atan2(R.get(1,0), R.get(0,0));
//		return new EulerAngles(new Angle(ANGULAR_UNITS.DEGREES, angle));
		double R31;
		Angle th;
		Angle gamma;
		Angle phi;
		boolean inSingularity = false;
		if(abs(R31) != 1 ){
			Angle th1 =-Math.asin(R31);
			Angle th2 = Math.pi - th1;
			Angle gamma1 = Math.atan2(R32/cos(th1), R33/cos(th1));
			Angle gamma2 = Math.atan2(R32/cos(th2), R32/cos(th2));
			Angle phi1   = Math.atan2(R21/cos(th1),R11/cos(th1));
			Angle phi2   = Math.atan2(R21/cos(th2),R21/cos(th2));
		}else{
			phi = 0;//can be any value in a singularity 
			if(R31 == -1){
				th = Math.pi/2
				gamma = phi+Math.atan2(R12, R13);
			}else{
				//R31 == 1
				th = -Math.pi/2;
				gamma = -phi+Math.atan2(-R12, -R13);
						
			}
		
		}
		EulerAngles result = EulerAngles(th,gamma,phi);
		if(inSingularity){
			result.setInSingularity();
		}
	
		return result;
		*/
		return null;
	}

	@Override
	/**
	 * TODO document 
	 * assuming if SO1 then only given x
	 * assuming if SO2 then only given x,y 
	 */
	public Quaternion toQuaternion() throws LogicException, CloneNotSupportedException {
		Number ZERO = new Double_Number(0.0);
		Number ONE = new Double_Number(1.0);
		
		//from http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche52.html
		//TODO think about effects of upcasting 
		//TODO speedup by extracting doubles earlier  
		Number r11 = thisMatrix.getNumCols() > 0 ? thisMatrix.getRow(1).getIndex(1) : ZERO;
		Number r12 = thisMatrix.getNumCols() > 0 ? thisMatrix.getRow(1).getIndex(2) : ZERO;
		Number r13 = thisMatrix.getNumCols() > 0 ? thisMatrix.getRow(1).getIndex(3) : ZERO;
		Number r21 = thisMatrix.getNumCols() > 1 ? thisMatrix.getRow(2).getIndex(1) : ZERO;			
		Number r22 = thisMatrix.getNumCols() > 1 ? thisMatrix.getRow(2).getIndex(2) : ZERO;
		Number r23 = thisMatrix.getNumCols() > 1 ? thisMatrix.getRow(2).getIndex(3) : ZERO;
		Number r31 = thisMatrix.getNumCols() > 2 ? thisMatrix.getRow(3).getIndex(1) : ZERO;
		Number r32 = thisMatrix.getNumCols() > 2 ? thisMatrix.getRow(3).getIndex(2) : ZERO;
		Number r33 = thisMatrix.getNumCols() > 2 ? thisMatrix.getRow(3).getIndex(3) : ZERO;
		
		Number q0 = (r11.add(r22).add(r33).add(ONE)).div(new Double_Number(4.0));
		Number q1 = (r11.sub(r22).sub(r33).sub(ONE)).div(new Double_Number(4.0));
		Number q2 = (r11.inverse().add(r22).sub(r33).add(ONE)).div(new Double_Number(4.0));
		Number q3 = (r11.inverse().sub(r22).add(r33).add(ONE)).div(new Double_Number(4.0));
		if(q0.isLess(ZERO)) q0 = ZERO;
		if(q1.isLess(ZERO)) q1 = ZERO;
		if(q2.isLess(ZERO)) q2 = ZERO;
		if(q3.isLess(ZERO)) q3 = ZERO;
		q0 = q0.sqrt();
		q1 = q1.sqrt();
		q2 = q2.sqrt();
		q3 = q3.sqrt();
		if((q0.isEqual(q1) || q0.isGreater(q1)) && (q0.isEqual(q2) || q0.isGreater(q2))&& (q0.isEqual(q3) || q0.isGreater(q3))){
		    q0 = q0.mult(ONE);
		    q1 = q1.mult((r32.sub(r23)).signum());
		    q2 = q2.mult((r13.sub(r31)).signum());
		    q3 = q3.mult((r21.sub(r12)).signum());
		} else if((q1.isEqual(q0) || q1.isGreater(q0)) && (q1.isEqual(q2) || q1.isGreater(q2)) && (q1.isEqual(q3) || q1.isGreater(q3))){ 
		    q0 = q0.mult((r32.sub(r23)).signum());
		    q1 = q1.mult(ONE);
		    q2 = q2.mult((r21.add(r12)).signum());
		    q3 = q3.mult((r13.add(r31)).signum());
		} else if((q2.isEqual(q0) || q2.isGreater(q0)) && (q2.isEqual(q1) || q2.isGreater(q1)) && (q2.isEqual(q3) || q2.isGreater(q3))){ 
		    q0 = q0.mult((r13.sub(r31)).signum());
		    q1 = q1.mult((r21.add(r12)).signum());
		    q2 = q2.mult(ONE);
		    q3 = q3.mult((r32.add(r23)).signum());
		} else if((q3.isEqual(q0) || q3.isGreater(q0)) && (q3.isEqual(q1) || q3.isGreater(q1)) && (q3.isEqual(q2) || q3.isGreater(q2))) {
		    q0 = q0.mult((r21.sub(r12)).signum());
		    q1 = q1.mult((r31.add(r13)).signum());
		    q2 = q2.mult((r32.add(r23)).signum());
		    q3 = q3.mult(new Double_Number(1.0));
		} else {
			throw new LogicException("quaternion conversion impossible state",MESSAGE_LEVEL.exception);
		}
		Number r = (q0.mult(q0).add(q1.mult(q1)).add(q2.mult(q2)).add(q3.mult(q3))).sqrt();
		q0 = q0.div(r);
		q1 = q1.div(r);
		q2 = q2.div(r);
		q3 = q3.div(r);
		return new Quaternion(q0, q1, q2, q3);
	}
	
	/**
	 * evaluets to a copy of the rotation matrix Scaled by a scaler S
	 * @param S
	 * @return
	 * @throws LogicException 
	 */
	public RotationMatrix scale(Number s) throws LogicException{
		Matrix M = thisMatrix.clone();
		M.scale(s);
		return new RotationMatrix(M);
	}
	

	/**
	 * evaluates to a rotationMatrix such that this*this.inverse() = I 
	 * @return the inverse of the rotationMatrix such that R*R.inverse = R.zero
	 */
	public RotationMatrix inverse(){
		//could also use R.inverse but R.transpose comes to the same result with n^2 run time 
		return new RotationMatrix(thisMatrix.transpose());
	}

	/**
	 * TODO document
	 * TODO implement
	 * @return
	 */
	public static RotationalRepersentation zero() {
		// TODO Auto-generated method stub
		return null;
	}
		
	}
	
	



