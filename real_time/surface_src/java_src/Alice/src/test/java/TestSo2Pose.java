import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.roboclub.robobuggy.map.So2Pose;


public class TestSo2Pose {

	@Before
	public void setUp() throws Exception {
	}

	@After
	public void tearDown() throws Exception {
	}

	@Test
	public void test() {
		So2Pose A = new So2Pose(0.0, 0.0, 0.0);
		So2Pose B = new So2Pose(0.0, 1.0, 0.0);
		So2Pose C = A.mult(B);
		if(C.getY() != 1.0){		
			fail("y value did not become 1.0 when it should ");
		}		
		
		//checking the inverse 
		C = A.mult(B.inverse());
		if(C.getY() != -1.0){		
			fail("y value did not become -1.0 when it should ");
		}
		
	}
	
	@Test
	public void test_equal(){
		So2Pose A = new So2Pose(1.0,2.0,3.0);
		So2Pose B = new So2Pose(1.0,2.0,3.0);
		if(!A.equals(B)){
			fail("So2Equal was not written correctly ");
		}
		
		if(!B.equals(A)){
			fail("So2Equal was not written correctly ");
		}
		
		So2Pose C = new So2Pose(1.1,2.0,3.0);
		if(C.equals(A)){
			fail("So2Equal was true when it should not be");
		}
	}
	
	@Test 
	public void test_inverse(){
		So2Pose A = new So2Pose(1.0,2.0,3.0);
		So2Pose B = A.inverse();

		So2Pose C = B.mult(A);
		if(!C.equals(So2Pose.Identity())){
			fail("matrix inverse time original is not identity");
		}
		
		
		So2Pose D = A.mult(B);
		if(!D.equals(So2Pose.Identity())){
			fail("matrix inverse time original is not identity");
		}
	}
	
	@Test
	public void test_2(){
		// test basic compounding of matrices
		So2Pose A = new So2Pose(0.0,1.0,0.0);
		So2Pose B = new So2Pose(0.0,2.0,0.0);
		So2Pose C = A.mult(B);
		
		if(C.getY() != 3.0){
			fail("matrix multiply is not implemented correctly ");
		}
		
		So2Pose D = B.mult(A);
		if(D.getY() != 3.0){
			fail("matrix multiply is not implemented correctly ");
		}
		
		//tests more complex matrix compounding 
		So2Pose E = new So2Pose(1.2,2.4,3.2);
		So2Pose F = new So2Pose(3.4,5.6,18);
		
		//in one order
		So2Pose G = E.mult(F);
		//in the other direction 
		So2Pose H = G.mult(F.inverse());  //should be the same as E

		/*
		if(E.getX() != H.getX() || E.getY() != H.getY() || E.getOrientation() != H.getOrientation()){
			fail("matrix multiply is not implemented correctly for some matrices ");
		}
		*/
	
		
	}

}
