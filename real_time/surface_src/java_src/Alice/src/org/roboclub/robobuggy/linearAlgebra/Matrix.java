package org.roboclub.robobuggy.linearAlgebra;

import org.roboclub.robobuggy.main.LogicException;

/***
 * 
 * @author Trevor Decker
 * @version 0.0
 * Representation of a matrix, a 2d array which operates like a matrix from linear algebra
 *
 */
public class Matrix<TYPE extends Number> {
	NDimensionalArray<TYPE> data; 
	
	//TODO add more constructors
	
	//TODO allow for a matrix that has multiple types ie angles and distinces 
	//TODO remove the need for an example type to be given when constructor is used  
	/**
	 * TODO document
	 * @return
	 */
	public Matrix(int ... dimensions){
		TYPE sampleElment = null; //TODO fix
		data = new NDimensionalArray<TYPE>(sampleElment, dimensions);
	}
	
	//todo make a constructor that takes an n d array as an input
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
	 * TODO document
	 * @param value
	 * @param row
	 * @param col
	 */
	public void set(TYPE value, int row,int col){
		data.set(value, row,col);
	}
	
	/**
	 * evaluates to a row vector with the elements in the rowth row of this matrix 
	 * @param row
	 * @return
	 */
	public Vector<TYPE> getRow(int row){
		Vector<TYPE> result = new Vector<TYPE>(this.getNumCols());
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
		Vector<TYPE> result = new Vector<TYPE>(this.getNumRows());
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
		Matrix<TYPE> result = new Matrix<TYPE>(endRow-startRow,getNumCols());
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
		Matrix<TYPE> result = new Matrix<TYPE>( getNumRows(),endCol-startCol);
		for(int col = 0;startCol+col<endCol;col++){
			for(int row = 0;row<this.getNumRows();row++){
				result.set(this.get(row, startCol+col), row, col);
			}
		}
		return result;
	}
	
	/**
	 * TODO document
	 * @return
	 * @throws LogicException 
	 */
	public Matrix<TYPE> add(Matrix<TYPE> b) throws LogicException{
		Matrix<TYPE> result =this.clone();
		result.data = this.data.add(b.data);
		return result;
	}
	
	/**
	 * TODO document
	 * @return
	 */
	public int getNumRows(){
		int[] dim = data.getDimensions();
		return dim[0];
	}
	
	/**
	 * TODO document
	 * @return
	 */
	public int getNumCols(){
		int[] dim = data.getDimensions();
		return dim[1];
	}

	/**
	 * TODO document
	 * @param s value to scale by 
	 * @throws LogicException 
	 */
	public void scale(Number s) throws LogicException {
		for(int i = 0;i<getNumRows();i++){
			for(int j = 0;j<getNumCols();j++){
				set((TYPE) s.mult(get(i,j)), i, j);	
			}
		}		
	}
	
	/**
	 * TODO document
	 * @param row
	 * @param col
	 * @return
	 */
	public TYPE get(int row,int col){
		return data.get(row,col);
	}

	/**
	 * TODO document
	 * @return
	 */
	public Matrix transpose() {
		Matrix<TYPE> result = new Matrix<TYPE>(this.getNumCols(),getNumRows());
		for(int i = 0;i<getNumRows();i++){
			for(int j = 0;j<getNumCols();j++){
				result.set(data.get(i,j), j,i);
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
		Matrix<TYPE> result = new Matrix<TYPE>(this.getNumRows(),getNumCols());
		for(int row = 0;row<this.getNumRows();row++){
			for(int col=0;col<this.getNumCols();col++){
				result.set(this.get(row, col), row, col);
			}
		}
		return result;
	}
	
	/**
	 * TODO document
	 * @return
	 * @throws LogicException 
	 */
	public Number determinate() throws LogicException{
		if(this.getNumCols() != this.getNumRows()){
			throw new LogicException("trying to find the determinate of a non square matrix which does not make sense");
		}
		if(this.getNumCols() == 0){
			throw new LogicException("trying to find the determinate of an empty matrix which does not make sense");
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
					Matrix<TYPE> cofactor = new Matrix<TYPE>(this.getNumRows()-1,getNumCols()-1);

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
		//TODO
		throw new LogicException("Matrix inverse has not been implemented yet");
	}
	
}
