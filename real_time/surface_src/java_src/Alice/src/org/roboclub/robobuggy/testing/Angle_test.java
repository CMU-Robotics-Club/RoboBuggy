package org.roboclub.robobuggy.testing;

import org.roboclub.robobuggy.linearAlgebra.ANGULAR_UNITS;
import org.roboclub.robobuggy.linearAlgebra.Angle;
import org.roboclub.robobuggy.linearAlgebra.Integer_Number;
import org.roboclub.robobuggy.main.LogicException;

import junit.framework.TestCase;

public class Angle_test extends TestCase {

	public void test1() throws LogicException{
		System.out.println("Starting to Test Angle");
		Angle a1 = new Angle(ANGULAR_UNITS.DEGREES,new Integer_Number(0));
		if(a1.getMeassurmentValue().toInteger_Number().getValue() != 0){
			fail();
		}
		Angle a2 = Angle.zero();
		if(a2.getMeassurmentValue().toInteger_Number().getValue() != 0){
			fail();
		}
		if(a1.equals(a2) != true){
			fail();
		}
		Angle a3 = new Angle(ANGULAR_UNITS.DEGREES,new Integer_Number(1));
		if(a2.getMeassurmentValue().toInteger_Number().getValue() != 1){
			fail();
		}
		//TODO add more tests, like a lot more tests
	}

}
