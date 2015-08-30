package org.roboclub.robobuggy.linearAlgebra;

import java.util.ArrayList;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.messages.GuiLoggingButton.LoggingMessage;
import org.roboclub.robobuggy.numbers.Number;


/***
 * @author Trevor Decker
 * @version 0.0
 *
 * Representation of a vector from linear algebra
 */
public class Vector<TYPE  extends Number> extends Matrix<TYPE> {	
	/**
	 * Creates a vector of length len (all values are not set)
	 * @param sampleElement
	 * @param len
	 */
	public Vector(TYPE sampleElement,int len) {
		super(sampleElement,len,1);
	}
		
	/**
	 * TODO document 
	 * @param input
	 * @return
	 */
	public Vector(TYPE sampleElement,ArrayList<TYPE> input){
		super(sampleElement,input.size(),1);
		for(int i = 0;i<input.size();i++){
			set(input.get(i), 1, i);
		}
	}
	
	/**
	 * TODO document
	 * @param values
	 * @throws LogicException 
	 */
	public  Vector(TYPE ... values) throws LogicException
	{
		super(values[0],values.length,1);

		if(values.length == 0){
			throw new LogicException("Can not create a zero length vector using this constructor, use 	public Vector(TYPE sampleElement,ArrayList<TYPE> input)instead", MESSAGE_LEVEL.exception);
		}
		
		for(int i = 0;i<values.length;i++){
			data.setIndex(i, values[i]);
		}
	}
	

	/**
	 * evaluates to the cross product of two vectors, requires that both vectors be of the same length
	 * @param otherVector
	 * @return
	 * @throws LogicException 
	 */
	public Number dotProduct(Vector<TYPE> otherVector,Number ZeroELement) throws LogicException{
		//TODO remove need for zero element to be passed in
		if(otherVector.getLength() != getLength()){
			throw new LogicException("Trying to take the dot product of two vectors of diffrent length",MESSAGE_LEVEL.exception);
		}
		
		Number result = ZeroELement;
		for(int i = 1;i<=getLength();i++){
			result = (TYPE) result.add(getIndex(i).mult(otherVector.getIndex(i)));
		}		
		return result;
	}
	
	/**
	 * evaluates to the number of elements in this vector
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
		return  new Vector<TYPE>(s1,s2,s3);
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
	 * @param index is 1 indexed  
	 * @return
	 */
	public TYPE getIndex(int index) {
		if(getNumCols() == 1){
			//is a row vector
			return data.get(index-1,0);
		}else{
			//is a col vector
			return data.get(0,index-1);
		}
	}
	
	/**
	 * converts this vector to an array list note index 
	 * 1 of the vector will become index 0 of the arraylist, 
	 * index 2 becomes index 0 .... to index n becoming index n -1
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
	 * @param index is 1 indexed 
	 * @param newValue
	 * @throws LogicException 
	 */
	public void setIndex(int index,TYPE newValue) throws LogicException{
		if(index > this.getLength()  || index < 1){
			throw new LogicException("index is out of range of vector",MESSAGE_LEVEL.exception);
		}
		if(this.getNumCols() == 1){
			//must be a row vector
			this.data.set(newValue, index-1,0);
		}else{
			//must be a col vectors
			this.data.set(newValue,0,index-1);
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
			Vector<TYPE> result = new Vector<TYPE>(sampleElement,this.getLength()+1);
			int i = 0;
			//i is not just in for loop so it can be used to set the last index's value to newValue
			for(;i<this.getLength();i++){
				result.setIndex(i, this.getIndex(i));
			}
			result.setIndex(i,newValue);
			return result;
	}
	
	/**
	 * TODO document
	 * @param M
	 * @return
	 * @throws LogicException 
	 */
	public static Vector fromMatrix(Matrix M) throws LogicException{
		if(M.getNumCols() == 1 ){
			//is a col vector
			return M.getCol(1);
		}else if(M.getNumRows() == 1){
			//is a row vector
			return M.getRow(1);
		}else{
			throw new LogicException("trying to make a matrix into a vector when it is not a vector",MESSAGE_LEVEL.exception);
		}
		

	}

}
