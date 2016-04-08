import com.roboclub.robobuggy.map.So2Pose;
import org.junit.Test;

import static org.junit.Assert.fail;


/**
 * Tests aspects of the So2Pose
 */
public class TestSo2Pose {


	/**
	 * basic test of the so2pose
	 */
	@Test
	public void test() {
		So2Pose a = new So2Pose(0.0, 0.0, 0.0);
		So2Pose b = new So2Pose(0.0, 1.0, 0.0);
		So2Pose c = a.mult(b);
		if(c.getY() != 1.0){
			fail("y value did not become 1.0 when it should ");
		}		
		
		//checking the inverse 
		c = a.mult(b.inverse());
		if(c.getY() != -1.0){
			fail("y value did not become -1.0 when it should ");
		}
		
	}

	/**
	 * tests whether 2 so2poses are equal
	 */
	@Test
	public void testEqual(){
		So2Pose a = new So2Pose(1.0,2.0,3.0);
		So2Pose b = new So2Pose(1.0,2.0,3.0);
		if(!a.equals(b)){
			fail("So2Equal was not written correctly ");
		}
		
		if(!b.equals(a)){
			fail("So2Equal was not written correctly ");
		}
		
		So2Pose c = new So2Pose(1.1,2.0,3.0);
		if(c.equals(a)){
			fail("So2Equal was true when it should not be");
		}
	}

	/**
	 * tests whether the inverse works
	 */
	@Test
	public void testInverse(){
		So2Pose a = new So2Pose(1.0,2.0,3.0);
		So2Pose b = a.inverse();

		So2Pose c = b.mult(a);
		if(!c.equals(So2Pose.identity())){
			fail("matrix inverse time original is not identity");
		}
		
		
		So2Pose d = a.mult(b);
		if(!d.equals(So2Pose.identity())){
			fail("matrix inverse time original is not identity");
		}
	}

	/**
	 * tests basic compounding of matrices
	 */
	@Test
	public void test2(){
		
		// test basic compounding of matrices
		So2Pose a = new So2Pose(0.0,1.0,0.0);
		So2Pose b = new So2Pose(0.0,2.0,0.0);
		So2Pose c = a.mult(b);
		
		if(c.getY() != 3.0){
			fail("matrix multiply is not implemented correctly ");
		}
		
		So2Pose d = b.mult(a);
		if(d.getY() != 3.0){
			fail("matrix multiply is not implemented correctly ");
		}
		
		//tests more complex matrix compounding 
		So2Pose e = new So2Pose(1.2,2.4,3.2);
		
		So2Pose f = new So2Pose(3.4,5.6,0);
		
		//in one order
		So2Pose g = e.mult(f);
		
		//in the other direction 
		So2Pose fInv = f.inverse();
		
		So2Pose h = g.mult(fInv);  //should be the same as E
		if(!e.equals(h)){
			fail("matrix multiply is not implemented correctly for some matrices ");
		}
	
		
	}

}
