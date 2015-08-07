package org.roboclub.robobuggy.linearAlgebra;

import java.lang.reflect.Array;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;

/***
 * 
 * @author Trevor Decker
 * @version 0.0
 * 
 * A way for creating arbitrary sized arrays of a generic type 
 * based on code from 	//http://stackoverflow.com/questions/4770926/java-n-dimensional-arrays
 *
 */
public class NDimensionalArray<CELL_TYPE  extends Number> {
	private int[] dimensions;
	private CELL_TYPE[] array;
	private int[] offsetMultipliers;
	
	/**
	 * Creates an N dimensional array of type given 
	 * smapleELment is required for the way that java allocates arrays dynamically at runtime. 
	 * The compiler does not know the type beforehand because of this. sampleElemnt should be 
	 * an initialized sample version of the element. 
	 * @param sampleElement
	 * @param dimensions
	 */
	@SuppressWarnings("unchecked")
	public NDimensionalArray(CELL_TYPE sampleElment,int ... dimensions) {
		//TODO remove need for sampleElment
		this.dimensions = dimensions;
		int arraySize = 1;
		for(int i =0;i<dimensions.length;i++){
			offsetMultipliers[i] = arraySize;
			arraySize *= dimensions[i];
		}
		
		array = (CELL_TYPE[]) Array.newInstance(sampleElment.getClass(), arraySize);
	}
	
	/**
	 * sets the cell that is pointed to by indices
	 * @param indices
	 * @return
	 */
	public CELL_TYPE get(int ... indices){
		return array[getOffset(indices)];
	}
	
	/***
	 * sets the cell that is pointed to by indices
	 * @param newValue
	 * @param indices
	 */
	public void set(CELL_TYPE newValue,int ... indices){
		array[getOffset(indices)] = newValue;
	}
	
	/**
	 * Gets the element in the array at the ith location in memory 
	 * if all elements were stored in a 1d row. 
	 */
	public CELL_TYPE getIndex(int index){
		return array[index];
	}
	
	/**
	 * Sets the element in the array at the ith location in memory if
	 * all elements were stored in a 1d row. 
	 */
	public void setIndex(int index,CELL_TYPE newValue){
		array[index] = newValue;
	}
	
	/**
	 * returns the dimensions of the matrix
	 * @return
	 */
	public int[] getDimensions(){
		return dimensions;
	}
	
	/***
	 * gets the offset for the cell at the indices given in the int[] indices 
	 * @param indices
	 * @return
	 */
	private int getOffset(int ... indices){
		int offset =0;
		for(int i = 0;i<dimensions.length;i++){
			offset += offsetMultipliers[i]*indices[i];
		}
		return offset;
	}
	
	/**
	 * requires that the Ndimensinal Array passed to this method has the same dimensions as
	 *  this NDimensinal array.   Evaluates to a new NdimensionalArray of the same dimensions as
	 *  this NdimensionalArray.  The value of each element in the new array is result of applying
	 *  the addition method to the corresponding index from this Array with the corresponding index's
	 *  value from the passed in array. 
	 *  @param B
	 * @return
	 * @throws LogicException 
	 */
	public NDimensionalArray<CELL_TYPE> add(NDimensionalArray<CELL_TYPE> b) throws LogicException{
		assert(this.getDimensions() == b.getDimensions()); //TODO check this
		NDimensionalArray<CELL_TYPE> result =this.clone();
		for(int i = 0;i<array.length;i++){
			@SuppressWarnings("unchecked")
			CELL_TYPE newValue = (CELL_TYPE) this.getIndex(i).add(b.getIndex(i));
			result.setIndex(i, newValue);
		}
		return result;
	}
	
	/**
	 * Creates a separate object that is equivalent to this object but stored in a different location in memory 
	 */
	public NDimensionalArray<CELL_TYPE> clone(){
		for(int i =0;i < this.dimensions.length;i++){
			if(this.dimensions[i] == 0){
				try {
					throw new LogicException("currently can not have clone a matrix with one demension 0",MESSAGE_LEVEL.exception);
				} catch (LogicException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
		int[] z = new int[this.dimensions.length];
		for(int i = 0;i<this.dimensions.length;i++){
			z[i] = 0;
		}
		
		CELL_TYPE sample = this.get(z);
		return new NDimensionalArray<CELL_TYPE>(sample,this.dimensions.clone());
	}
	
}
