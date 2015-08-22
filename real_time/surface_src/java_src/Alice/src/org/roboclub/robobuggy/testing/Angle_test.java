package org.roboclub.robobuggy.testing;

import org.roboclub.robobuggy.main.LogicException;
import org.roboclub.robobuggy.measurments.ANGULAR_UNITS;
import org.roboclub.robobuggy.measurments.Angle;
import org.roboclub.robobuggy.numbers.Integer_Number;
import org.roboclub.robobuggy.numbers.Number;

import junit.framework.TestCase;

public class Angle_test extends TestCase {

	public void test1() throws LogicException{
		System.out.println("Starting to Test Angle equality Test");
		Angle a1 = new Angle(ANGULAR_UNITS.DEGREES,new Integer_Number(0));
		if(((Number) a1.getMeassurmentValue()).toInteger_Number().getValue() != 0){
			fail();
		}
		Angle a2 = Angle.zero();
		if(((Number) a2.getMeassurmentValue()).toInteger_Number().getValue() != 0){
			fail();
		}
		if(a1.equals(a2) != true){
			fail();
		}
		Angle a3 = new Angle(ANGULAR_UNITS.DEGREES,new Integer_Number(1));
		if(((Number) a2.getMeassurmentValue()).toInteger_Number().getValue() != 1){
			fail();
		}
	}
	
	//TODO add more tests, like a lot more tests
	public void test2() throws LogicException, CloneNotSupportedException{
		System.out.println("starting more Test for Angle equality");
		Angle a1 = new Angle<Integer_Number>(ANGULAR_UNITS.DEGREES, new Integer_Number(100));
		System.out.println(a1);
		System.out.println(a1.signum());
		System.out.println("done");

		//TODO
	}

	

}
