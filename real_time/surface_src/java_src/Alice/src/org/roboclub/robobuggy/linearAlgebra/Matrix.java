package org.roboclub.robobuggy.linearAlgebra;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.main.MESSAGE_LEVEL;
import org.roboclub.robobuggy.numbers.Number;

/***
 * 
 * @author Trevor Decker
 * @version 0.0
 * Representation of a matrix, a 2d array which operates like a matrix from linear algebra
 *
 */
public class Matrix<TYPE extends Number> {
	NDimensionalArray<TYPE> data; 
	TYPE sampleElement;
		
	//TODO allow for a matrix that has multiple types ie angles and distinces 
	//TODO remove the need for an example type to be given when constructor is used  
	/**
	 * TODO document
	 * @return
	 */
	public Matrix(TYPE sampleElment,int ... dimensions){
		this.sampleElement = sampleElment;
		data = new NDimensionalArray<TYPE>(sampleElment, dimensions);
	}
	
	/**
	 * TODO document 
	 * TODO implement
	 * @param rows
	 * @param cols
	 * @param values
	 * @throws LogicException 
	 */
	public Matrix(int rows, int cols, TYPE ... values) throws LogicException{
		if(values.length == 0){
			throw new LogicException("need at least one value to generate a matrix with this constructor",MESSAGE_LEVEL.exception);
		}
		this.sampleElement =  values[0];
		//TODO finish implemention
		
	}
	
/**
 * TODO document
 * TODO implement
 * @param input
 */
	public Matrix(Number[][] input){
		//TODO
	}
	
	
	/**
	 * TODO document
	 * @return
	 * @throws LogicException 
	 */
	public Matrix<TYPE> mult(Matrix<TYPE> otherMatrix) throws LogicException{
		Matrix<TYPE> result = this.clone();
		for(int x = 0;x<getNumCols();x++)
		{
			for(int y = 0;y<getNumRows();y++)
			{
				result.set(getRow(x).mult(otherMatrix.getCol(y)),x,y);
			}
		}
				
		return null;
	}
	
	/**
	 * This method has side effects
	 * It sets the matrix (col,row) to value. Overriding the value that use to be at the adress
	 * @param value
	 * @param row
	 * @param col
	 */
	public void set(TYPE value, int row,int col){
		data.set(value, col-1,row-1);
	}
	
	/**
	 * evaluates to a row vector with the elements in the rowth row of this matrix 
	 * @param row
	 * @return
	 */
	public Vector<TYPE> getRow(int row){
		Vector<TYPE> result = new Vector<TYPE>(sampleElement,this.getNumCols());
		for(int i = 0;i<this.getNumCols();i++){
			result.set(this.get(row, i), 0, i);
		}
		return result;
	}
	
	/**
	 * evaluates to a column vector with the elements in the colth column of this matrix 
	 * @param col
	 * @return
	 */
	public Vector<TYPE> getCol(int col){
		Vector<TYPE> result = new Vector<TYPE>(sampleElement,this.getNumRows());
		for(int i = 0;i<this.getNumRows();i++){
			result.set(this.get(i, col), i, 0);
		}
		return result;
	}
	
	/**
	 * evaluates to a matrix equivalent to the subsection of this matrix including all of rows startRow to endRow 
	 * @param startRow
	 * @param endRow
	 * @return
	 */
	public Matrix<TYPE> getRows(int startRow,int endRow){
		Matrix<TYPE> result = new Matrix<TYPE>(this.sampleElement,endRow-startRow,getNumCols());
		for(int row = 0;startRow+row<endRow;row++){
			for(int col = 0;col<this.getNumCols();col++){
				result.set(this.get(startRow+row, col), row, col);
			}
		}
		return result;
	}
	
	/**
	 * evaluates to a matrix equivalent to the subsection of this matrix including all of columns startCol to endCol 
	 * @param startCol
	 * @param endCol
	 * @return
	 */
	public Matrix<TYPE> getCols(int startCol,int endCol){
		Matrix<TYPE> result = new Matrix<TYPE>(this.sampleElement,getNumRows(),endCol-startCol);
		for(int col = 0;startCol+col<endCol;col++){
			for(int row = 0;row<this.getNumRows();row++){
				result.set(this.get(row, startCol+col), row, col);
			}
		}
		return result;
	}
	
	/**
	 * evaluates to the matrix you would get by adding this matrix with 
	 *  matrix b.  
	 *  If matrix b and this matrix are of different sizes then a logic error will be thrown
	 * @return
	 * @throws LogicException 
	 */
	public Matrix<TYPE> add(Matrix<TYPE> b) throws LogicException{
		Matrix<TYPE> result =this.clone();
		result.data = this.data.add(b.data);
		return result;
	}
	
	/**
	 * Evaluates to the number of rows in this matrix 
	 * @return
	 */
	public int getNumRows(){
		int[] dim = data.getDimensions();
		return dim[0];
	}
	
	/**
	 * Evaluates to the number of columns in this matrix
	 * @return
	 */
	public int getNumCols(){
		int[] dim = data.getDimensions();
		return dim[1];
	}

	/**
	 * evaluates to a copy of this matrix with all values scaled by s
	 *           a transform between the current matrix number system
	 *           and the number type of S must exist 
	 * @param s value to scale by 
	 * @throws LogicException 
	 */
	public Matrix<TYPE> scale(Number s) throws LogicException {
		Matrix<TYPE> result = new Matrix<TYPE>(this.sampleElement,this.getNumCols(),getNumRows());
		for(int col = 1;col<=getNumCols();col++){
			for(int row = 1;row<=getNumRows();row++){
			//	result.set((TYPE) s.mult(get(col,row)), col, row);	
			}
		}
		return result;
	}
	
	/**
	 * Evaluates to the element at (col,row) of the matrix 
	 * @param row
	 * @param col
	 * @return
	 */
	public TYPE get(int row,int col){
		//data is 0 indexed while matrix is 1 indexed
		return data.get(col-1,row-1);
	}

	/**
	 * Evaluates to the transpose of the matrix Ie a nxm matrix becomes mxn
	 * @return
	 */
	public Matrix transpose() {
		Matrix<TYPE> result = new Matrix<TYPE>(this.sampleElement,this.getNumCols(),getNumRows());
		for(int i = 1;i<=getNumRows();i++){
			for(int j = 1;j<=getNumCols();j++){
				result.set(get(j,i), j,i);
			}
		}	
		return result;
	}
	
	/**
	 * creates a new matrix with the same values as this one but as a completely separate reference
	 * NOTE is a shallow copy ie will if it is a matrix of pointers then the copy matrix will have the 
	 * the same pointers in its cells 
	 * @return
	 */
	@Override
	public Matrix<TYPE> clone(){
		Matrix<TYPE> result = new Matrix<TYPE>(this.sampleElement,this.getNumRows(),getNumCols());
		for(int row = 0;row<this.getNumRows();row++){
			for(int col=0;col<this.getNumCols();col++){
				result.set(this.get(row, col), row, col);
			}
		}
		return result;
	}
	
	/**
	 * Calculates the determinate of this matrix
	 *    will return a logException if the matrix is not square 
	 * @return
	 * @throws LogicException 
	 * @throws CloneNotSupportedException 
	 */
	public Number determinate() throws LogicException, CloneNotSupportedException{
		if(this.getNumCols() != this.getNumRows()){
			throw new LogicException("trying to find the determinate of a non square matrix which does not make sense",MESSAGE_LEVEL.exception);
		}
		if(this.getNumCols() == 0){
			throw new LogicException("trying to find the determinate of an empty matrix which does not make sense",MESSAGE_LEVEL.exception);
		}
		//1x1 case
		if(this.getNumRows() == 1){
			return this.get(0, 0);
		}
		//2x2 case
		if(this.getNumRows() == 2){
			return get(0,0).mult(get(0,1)).sub(get(1,0).mult(get(0,1)));
		}
		//nxn case
		TYPE result = (TYPE) this.get(1, 1).getZero();
		Number A = (TYPE) this.get(1,1).getOne();
		for(int i = 0;i<this.getNumRows();i++){
				for(int col =1;col<this.getNumCols();col++){
					Matrix<TYPE> cofactor = new Matrix<TYPE>(this.sampleElement,this.getNumRows()-1,getNumCols()-1);

					//Populates the cofactor skipping the 1st col and the ith row
					for(int row = 0;row<i;row++){
						cofactor.set(this.get(row, col), row, col-1);
					}
					for(int row = i+1;row<this.getNumRows();row++){
						cofactor.set(this.get(row, col), row-1, col-1);
					}
					result = (TYPE) A.mult(result.add(cofactor.determinate()));	
				}
				A = A.inverse();
		}
		return result;
	}
	
	/**
	 * TODO implement
	 * TODO document
	 * @return
	 * @throws LogicException 
	 */
	public Matrix<TYPE> inverse() throws LogicException{
		throw new LogicException("Matrix inverse has not been implemented yet",MESSAGE_LEVEL.exception);
	}
	
	@Override
	/**
	 * Creates a human readable string representation of the matrix,  is designed for displaying the matrix 
	 *  in a text box,log file, or print statement
	 */
	public String toString(){
		String result = "";
		for(int row = 1;row<=getNumRows();row++){
			for(int col = 1;col<=getNumCols();col++){
				result+=get(col, row).toString();
				if(col < getNumCols()){
					result+=",";
				}
			}
			if(row < getNumRows()){
				result+= "\n";
			}
		}
		return result;
	}
	
}
