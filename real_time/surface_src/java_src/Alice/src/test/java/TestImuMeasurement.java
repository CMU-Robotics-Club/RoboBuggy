import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.roboclub.robobuggy.messages.ImuMeasurement;


public class TestImuMeasurement {

	@Before
	public void setUp() throws Exception {
	}

	@After
	public void tearDown() throws Exception {
	}

	@Test
	public void test() {
		double roll = 0.123;
		double pitch = 4.567;
		double yaw = 8.9;
		ImuMeasurement aMeasurement = new  ImuMeasurement(yaw,pitch,roll);
		if(aMeasurement.getPitch() != pitch ){
			fail("pitch does not equal pitch");
		}
		
		if(aMeasurement.getRoll() != roll ){
			fail("roll does not equal roll");
		}		
		
		if(aMeasurement.getYaw() != yaw ){
			fail("yaw does not equal yaw");
		}
	
		
		fail("Not yet implemented");
	}

}
