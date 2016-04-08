import com.roboclub.robobuggy.messages.ImuMeasurement;
import org.junit.Test;

import static org.junit.Assert.fail;


/**
 * tests the imu's measurement mathematics
 */
public class TestImuMeasurement {

	/**
	 * tests the imu's math
	 */
	@Test
	public void test() {
		double roll = 0.123;
		double pitch = 4.567;
		double yaw = 8.9;
		ImuMeasurement aMeasurement = new  ImuMeasurement(yaw,pitch,roll);
		if(Math.abs(aMeasurement.getPitch() - pitch) > .0001 ){
			fail("pitch does not equal pitch");
		}
		
		if(Math.abs(aMeasurement.getRoll() - roll) >.0001 ){
			fail("roll does not equal roll");
		}		
		
		if(Math.abs(aMeasurement.getYaw() - yaw) >.0001 ){
			fail("yaw does not equal yaw");
		}
	
		
	}

}
