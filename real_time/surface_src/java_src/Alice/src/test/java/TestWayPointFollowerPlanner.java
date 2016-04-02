import com.roboclub.robobuggy.messages.GPSPoseMessage;
import org.junit.Test;

import java.util.Date;

import static org.junit.Assert.fail;


/**
 *
 */
public class TestWayPointFollowerPlanner {


	/**
	 * Tests the waypoint follower by giving it different poses
	 */
	@Test
	public void test() {
		GPSPoseMessage zeroPose = new GPSPoseMessage(new Date(), 0.0, 0.0, 0.0);
		GPSPoseMessage onePose = new GPSPoseMessage(new Date(), 0, 1, 0);
		GPSPoseMessage aPose = new GPSPoseMessage(new Date(), 1.5, 1, 0);
		GPSPoseMessage negPose = new GPSPoseMessage(new Date(), 0, -1, 0);



		if(Math.abs(GPSPoseMessage.getDistance(zeroPose, zeroPose) - 0) > .001){
			fail("distance function does not respect idenity");
		}
		
		if(Math.abs(GPSPoseMessage.getDistance(zeroPose, onePose) - 1.0) > .001){
			fail("distance function does not handle some input correctly ");
		}
		
		if(Math.abs(GPSPoseMessage.getDistance(aPose, zeroPose) -  1.802) > .001){
			fail("distance function does not handle some input correctly ");
		}
		
		if(Math.abs(GPSPoseMessage.getDistance(onePose, aPose) - 1.5) > .001){
			fail("distance function does not handle some input correctly ");
		}
		
		if(Math.abs(GPSPoseMessage.getDistance(onePose, negPose) - 2.0) > .001){
			fail("distance function does not handle some input correctly ");
		}
		
		
		
		}

	}
