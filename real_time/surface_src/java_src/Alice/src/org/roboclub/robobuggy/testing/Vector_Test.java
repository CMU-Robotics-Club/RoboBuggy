package org.roboclub.robobuggy.testing;

import org.roboclub.robobuggy.linearAlgebra.Vector;
import org.roboclub.robobuggy.numbers.Integer_Number;

import junit.framework.TestCase;

public class Vector_Test extends TestCase {
	public void Assert(boolean result){
		if(result == false){
			System.out.println("Test Failed");
			fail();
		}else{
			System.out.println("Test Passed");
		}
	}
	
	public void test1() throws Exception{
		System.out.println("Starting to test Vector");
		Vector<Integer_Number> axis1 = new Vector<Integer_Number>(Integer_Number.zero(),3);
		axis1.setIndex(1, new Integer_Number(1));
		axis1.setIndex(2, new Integer_Number(2));
		axis1.setIndex(3, new Integer_Number(3));
		Vector<Integer_Number> axis2 = new Vector(new Integer_Number(1),new Integer_Number(2),new Integer_Number(3));
		Vector<Integer_Number> axis3 = new Vector(new Integer_Number(1),new Integer_Number(0),new Integer_Number(0));

		Integer_Number resultInteger = (Integer_Number) axis1.dotProduct(axis2, Integer_Number.zero());
		Assert(resultInteger.isEqual(new Integer_Number(14)));
		System.out.println(axis1.toString());
		System.out.println(axis1.transpose().toString());         //TODO make an assert
 		System.out.println(axis1.crossProduct(axis3).toString()); //TODO make an assert
		
		
	//	Assert(axis1.equals(axis2));
		//TODO wrtie more tests
		System.out.println("Finished Testing Vector");
	}
}
