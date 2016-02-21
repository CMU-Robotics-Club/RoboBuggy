package com.roboclub.robobuggy.main;

import Jama.Matrix;

/**
 * Util - Matrix utilities and other utilities
 */
public final class Util {

	/**
	 * creates the identity matrix
	 * @param size size of the matrix
	 * @return the identity matrix
	 */
	public static Matrix createIdentityMatrix(int size){
		Matrix result = new Matrix(size,size,0.0);
		for(int i = 0;i<size;i++){
			result.set(i,i,1.0);
		}
		return result;
	}

	
}
