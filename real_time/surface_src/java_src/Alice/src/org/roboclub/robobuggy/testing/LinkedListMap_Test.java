package org.roboclub.robobuggy.testing;

import org.roboclub.robobuggy.coordinateFrame.FrameOfRefrence;
import org.roboclub.robobuggy.coordinateFrame.Pose;
import org.roboclub.robobuggy.coordinateFrame.PositionRepersentation;
import org.roboclub.robobuggy.coordinateFrame.RotationMatrix;
import org.roboclub.robobuggy.linearAlgebra.Matrix;
import org.roboclub.robobuggy.linearAlgebra.Vector;
import org.roboclub.robobuggy.map.LinkedListMap;
import org.roboclub.robobuggy.map.Point;
import org.roboclub.robobuggy.measurments.DISTINCE_UNITS;
import org.roboclub.robobuggy.measurments.Distince;
import org.roboclub.robobuggy.numbers.Double_Number;

import junit.framework.TestCase;

public class LinkedListMap_Test extends TestCase {

	public void test1() throws Exception{
		LinkedListMap map1 = new LinkedListMap();
		//TODO make it much simpler to create a point (add convince constructors) this is ridiculous
		Distince sampleDistince = new Distince(DISTINCE_UNITS.METERS,Double_Number.zero());
		Vector<Distince> coordinates = new Vector<Distince>(sampleDistince,3);
		coordinates.set(new Distince(DISTINCE_UNITS.METERS, new Double_Number(0.0)),1,1);
		coordinates.set(new Distince(DISTINCE_UNITS.METERS, new Double_Number(0.0)),1,2);
		coordinates.set(new Distince(DISTINCE_UNITS.METERS, new Double_Number(0.0)),1,3);		
		org.roboclub.robobuggy.coordinateFrame.Point position = new org.roboclub.robobuggy.coordinateFrame.Point(coordinates);
		Double_Number sample_doubleNumber = Double_Number.zero();
		Matrix<Double_Number> Matrix_D = new Matrix<Double_Number>(sample_doubleNumber,3,3);
		Matrix_D.set(new Double_Number(1.0), 1, 1);
		Matrix_D.set(new Double_Number(0.0), 1, 2);
		Matrix_D.set(new Double_Number(0.0), 1, 3);
		Matrix_D.set(new Double_Number(0.0), 2, 1);
		Matrix_D.set(new Double_Number(1.0), 2, 2);
		Matrix_D.set(new Double_Number(0.0), 2, 3);
		Matrix_D.set(new Double_Number(0.0), 3, 1);
		Matrix_D.set(new Double_Number(0.0), 3, 2);
		Matrix_D.set(new Double_Number(1.0), 3, 3);
		RotationMatrix<Double_Number> R = new RotationMatrix<Double_Number>(Matrix_D);
		Pose pose1 = new Pose(position, R);
		Point p1 = new Point(pose1);
		map1.AddObject(p1);
		//TODO write more tests


	}
}
