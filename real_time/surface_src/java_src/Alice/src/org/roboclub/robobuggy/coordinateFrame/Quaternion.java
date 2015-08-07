package org.roboclub.robobuggy.coordinateFrame;

import org.roboclub.robobuggy.linearAlgebra.Double_Number;
import org.roboclub.robobuggy.linearAlgebra.Number;
import org.roboclub.robobuggy.linearAlgebra.Vector;
import org.roboclub.robobuggy.main.LogicException;

/***
 * for more info look around pg 33 of 	http://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-complete.pdf
 * @author Trevor Decker
 * @version 0.0
 * 
 * TODO document 
 * TODO implement
 *
 */
public class Quaternion<NTYPE extends Number> implements RotationalRepersentation{
	NTYPE q0;
	Vector<NTYPE> qv;
	

	/**
	 * constructor for a quaterinon being passed in the scale(q0) and rotation vector qv
	 * @param q0
	 * @param qV
	 */
	public Quaternion(NTYPE q0, Vector<NTYPE> qV) {
		this.q0 = q0;
		this.qv = qV;
	}

	/**
	 * TODO document 
	 * @param q02
	 * @param q1
	 * @param q2
	 * @param q3
	 * @throws LogicException 
	 */
	public Quaternion(NTYPE q02, NTYPE q1, NTYPE q2, NTYPE q3) throws LogicException {
		this.q0 = q0; //magnitude 
		this.qv = new Vector<NTYPE>(3);
		this.qv.setIndex(0, q1);
		this.qv.setIndex(1, q2);
		this.qv.setIndex(1, q3);
	}

	/**
	 * produces a new Quaternion that is the conjugate of this Quaternion 
	 * @return
	 * @throws LogicException 
	 */
	public Quaternion conjugate() throws LogicException{
		return new Quaternion(q0,qv.getIndex(0).inverse(),qv.getIndex(1).inverse(),qv.getIndex(2).inverse());
	}
	
	/**
	 * computes the reciprical of this quaterion this*this.reciprical() = identity quatrinon 
	 * @return
	 * @throws LogicException 
	 */
	public Quaternion reciprical() throws LogicException{
		
		return new Quaternion(this.q0.div(this.norm().mult(this.norm())),this.conjugate().qv);
	}
	
	
	/**
	 * computes the norm of this quaternion 
	 * @return
	 * @throws LogicException 
	 */
	public NTYPE norm() throws LogicException{
		return (NTYPE) (q0.mult(q0).add(qv.dotProduct(qv,q0.getZero()))).sqrt();
	}
	
	/***
	 * computes this + Q, the independent element wise addition of the scale and vector components 
	 * @param Q
	 * @return
	 * @throws LogicException 
	 */
	public Quaternion addition(Quaternion Q) throws LogicException{
		return new Quaternion(q0.add(Q.q0), (Vector) qv.add(Q.qv));
	}
	
	/*** 
	 * Scales the rotation of this quaterinon by a double 
	 * @param scaler
	 * @return
	 * @throws LogicException 
	 */
	public Quaternion ScalerMultiplication(Number scaler) throws LogicException{
		return new Quaternion (scaler.mult(q0),qv.getIndex(0),qv.getIndex(1),qv.getIndex(2));
	}
	
	
	/**
	 * helper function which scales a vector/matrix by a double
	 * @param scale
	 * @param A
	 * @return
	 * @throws LogicException 
	 */
	private Vector scaleVector(NTYPE scale,Vector A) throws LogicException{
		Vector B = (Vector) A.clone();
		for(int i = 0;i<A.getLength();i++){
			B.setIndex(i,scale.mult(A.getIndex(i)));
		}
		return B;
	}
	
	/**
	 * result of Q*P  where Q is this quaternion and P is another quaternion  
	 * @param P
	 * @return
	 * @throws LogicException 
	 */
	public Quaternion QuaternionMultiplication(Quaternion<NTYPE> P) throws LogicException{
		NTYPE c0 = (NTYPE) q0.mult(P.q0).add(qv.dotProduct(P.qv,q0.getZero()).inverse());
		Vector<NTYPE> C = (Vector) scaleVector(q0,P.qv).add(scaleVector(P.q0, qv)).add(qv.crossProduct(P.qv));				
		return new Quaternion(c0,C);
	}
	
	/**
	 * TODO implement
	 * TODO document
	 * evaluates to a rotation matrix in so3
	 * @return
	 */
	public RotationMatrix toRotationMatrix(){

		
		return null;
	}
	
	/**
	 * TODO implement
	 * TODO document
	 * @return
	 */
	public EulerAngles toEulerAngles(){
		return null;
	}

	@Override
	/**
	 * TODO document 
	 */
	public RotationalRepersentation preApplyRotation(RotationalRepersentation secondRotation) throws LogicException, CloneNotSupportedException {
		return this.addition(secondRotation.toQuaternion());
	}

	@Override
	/**
	 * TODO document
	 */
	public RotationalRepersentation postApplyRotation(
			RotationalRepersentation secondRotation) throws LogicException, CloneNotSupportedException {
		return secondRotation.preApplyRotation(this);
	}

	@Override
	/**
	 * TODO document 
	 */
	public Quaternion toQuaternion() {
		return this;
	}

	/**
	 * Evaluates to the zero quaternion  
	 * TODO only works for Double_Numbers right now, make work for any number type
	 * @return
	 * @throws LogicException 
	 */
	public static RotationalRepersentation zero() throws LogicException {
		return new Quaternion<Number>(Double_Number.zero(), Double_Number.zero(), Double_Number.zero(), Double_Number.zero());
	}
	
	
	
	
}
