package com.roboclub.robobuggy.main;

import Jama.Matrix;

public final class Util {

	public static Matrix eye(int size){
		Matrix result = new Matrix(size,size,0.0);
		for(int i = 0;i<size;i++){
			result.set(i,i,1.0);
		}
		return result;
	}
	
}
