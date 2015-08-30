package org.roboclub.robobuggy.coordinateFrame2;

/**
 * 
 * @author Trevor Decker
 * This is a simplistic version of the coordinate frame system where all data is encoded 
 * in a state matrix. It should eventually be replaced by a more complete coordinateFrame system
 */
public class StateMatrix {
	//A list of the states what coordinate is for which state vector
	//A matrix showing the state dynamics
	int rows = 0;
	int cols = 0;
	double[][] matrix;
	States[] states;
	
	public StateMatrix(double[][] matrix,States[] states){
		//TODO constructor
	}
	
	//aka multiply on the left
	public StateMatrix preApply(StateMatrix otherMatrix){
		//TODO figure out how to permute the state matrix since the states are in a diffrent order
		//TODO check to make sure that the states are of the correct type 
		double[][] resultMatrix = new double[this.rows][otherMatrix.cols];
		for(int row = 0;row<this.rows;row++){
			for(int col = 0;col<otherMatrix.cols;col++){
				//thisRow * otherCol
				resultMatrix[row][col] = 0;
				for(int i = 0;i<this.cols;i++){
					resultMatrix[row][col] += this.matrix[row][i]*otherMatrix.matrix[i][col];
				}
			}
		}
		return new StateMatrix(resultMatrix,states);
	}
	
	//aka multiply on the right
	public StateMatrix postApply(StateMatrix otherMatrix){
		return otherMatrix.preApply(this);
	}
	
	public StateVector postApply(StateVector X){
		//assumed that stateVector is a column vector
		//assuming that the order of the states is the same 
		double[] result = new double[X.states.length];
		for(int row = 0;row<X.states.length;row++){
			result[row] = 0;
			for(int col = 0;col<this.cols;col++){
				result[row] += matrix[row][col]*X.vector[row];
			}
		}
		return new StateVector(result,X.states);
	}
	
}
