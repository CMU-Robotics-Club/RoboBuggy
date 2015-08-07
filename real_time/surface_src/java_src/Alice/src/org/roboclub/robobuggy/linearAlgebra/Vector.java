package org.roboclub.robobuggy.linearAlgebra;

import java.util.ArrayList;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;


/***
 * @author Trevor Decker
 * @version 0.0
 *
 * Representation of a vector from linear algebra
 */
public class Vector<TYPE  extends Number> extends Matrix<TYPE> {	
	/**
	 * TODO document
	 * @param sampleElment
	 * @param len
	 */
	public Vector(int len) {
		super(len);
	}
	
	/**
	 * TODO document 
	 * @param input
	 * @return
	 */
	public Vector(TYPE sampleElment,ArrayList<TYPE> input){
		super(input.size());
		for(int i = 0;i<input.size();i++){
			set(input.get(i), 0, i);
		}
	}
	
	/**
	 * TODO document
	 * @param sampleElement
	 * @param values
	 */
	public  Vector(TYPE sampleElement,TYPE ... values)
	{
		super(values.length);
		for(int i = 0;i<values.length;i++){
			data.setIndex(i, values[i]);
		}
	}
	

	/**
	 * TODO document  
	 * @param otherVector
	 * @return
	 * @throws LogicException 
	 */
	public Number dotProduct(Vector<TYPE> otherVector,Number ZeroELement) throws LogicException{
		if(otherVector.getLength() != getLength()){
			throw new LogicException("Trying to take the dot product of two vectors of diffrent length",MESSAGE_LEVEL.exception);
		}
		
		Number result = ZeroELement;
		for(int i = 0;i<getLength();i++){
			result = (TYPE) result.add(getIndex(i).mult(otherVector.getIndex(i)));
		}		
		return result;
	}
	
	/**
	 * TODO document
	 * @return
	 */
	public int getLength(){
		return Math.max(getNumCols(),getNumRows());
	}
	
	
	/**
	 * TODO document
	 * @param otherVector
	 * @return
	 * @throws LogicException 
	 */
	public Vector<TYPE> crossProduct(Vector<TYPE> otherVector) throws LogicException{
		//calculate the cross product 
		TYPE s1 = (TYPE) this.getIndex(2).mult(otherVector.getIndex(3)).sub(this.getIndex(3).mult(otherVector.getIndex(2)));
		TYPE s2 = (TYPE) this.getIndex(3).mult(otherVector.getIndex(1)).sub(this.getIndex(1).mult(otherVector.getIndex(3)));
		TYPE s3 = (TYPE) this.getIndex(1).mult(otherVector.getIndex(2)).sub(this.getIndex(2).mult(otherVector.getIndex(1)));
		//pack and return the result 
		return  new Vector<TYPE>(s1, s1,s2,s3);
	}
	
	/**
	 * TODO document
	 * @param b
	 * @return
	 * @throws LogicException 
	 */
	public TYPE mult(Vector<TYPE> b) throws LogicException{
		if(this.getNumRows() == 0){
			throw new LogicException("it does not make sense to multiply a zero size vector",MESSAGE_LEVEL.exception);
		}
		//the vector has at least one element in it 
		TYPE result = this.getIndex(0);
		if(this.getNumRows() != b.getNumCols()){
			throw new LogicException("dimension mismatch for vector multiplication",MESSAGE_LEVEL.exception);
		}
		if(this.getNumCols() != 1){
			throw new LogicException("dimension mismatch for vector multiplication",MESSAGE_LEVEL.exception);
		}
		
		if(b.getNumRows() != 1){
			throw new LogicException("dimension mismatch for vector multiplication",MESSAGE_LEVEL.exception);
		}
		
		for(int i = 0;i<b.getLength();i++){
			result = (TYPE) result.add(this.get(0, i).mult(b.get(i, 0)));
		}
		return result;
	}

	/**
	 * TODO document 
	 * @param i
	 * @return
	 */
	public TYPE getIndex(int i) {
		return data.get(i);
	}
	
	/**
	 * TODO document 
	 * @return
	 */
	public ArrayList<TYPE> toArrayList(){
		ArrayList<TYPE> result = new ArrayList<TYPE>();
		for(int i = 0;i<getLength();i++){
			result.add(this.getIndex(i));
		}
		return result;
	}
	
	/**
	 * TODO document
	 * @param index
	 * @param newValue
	 * @throws LogicException 
	 */
	public void setIndex(int index,TYPE newValue) throws LogicException{
		if(this.getLength() > index || index > -1){
			throw new LogicException("index is out of range of vector",MESSAGE_LEVEL.exception);
		}
		if(this.getNumCols() == 1){
			//must be a row vector
			this.data.set(newValue, 0,index);
		}else{
			//must be a col vectors
			this.data.set(newValue,index,0);
		}
	}
	
	/**
	 * evaluates to a new vector that has the same contents of this vector with a new value 
	 * appended on to the end 
	 * Currently evaluates in linear time
	 * @param newValue
	 * @return 
	 * @throws LogicException 
	 */
	public Vector<TYPE> append(TYPE newValue) throws LogicException{
			Vector<TYPE> result = new Vector<TYPE>(this.getLength()+1);
			int i = 0;
			//i is not just in for loop so it can be used to set the last index's value to newValue
			for(;i<this.getLength();i++){
				result.setIndex(i, this.getIndex(i));
			}
			result.setIndex(i,newValue);
			return result;
	}

}
