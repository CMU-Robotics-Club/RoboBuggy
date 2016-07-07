import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.nodes.localizers.LocalizerUtil;
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


        if (Math.abs(GPSPoseMessage.getDistance(zeroPose, zeroPose) - 0) > .001) {
            fail("distance function does not respect idenity");
        }

        if (Math.abs(GPSPoseMessage.getDistance(zeroPose, onePose) - LocalizerUtil.convertLonToMeters(1.0)) > .001) {
            fail("distance function does not handle some input correctly ");
        }

        if (Math.abs(GPSPoseMessage.getDistance(onePose, aPose) - LocalizerUtil.convertLatToMeters(1.5)) > .001) {
            fail("distance function does not handle some input correctly ");
        }

        if (Math.abs(GPSPoseMessage.getDistance(onePose, negPose) - LocalizerUtil.convertLonToMeters(2.0)) > .001) {
            fail("distance function does not handle some input correctly ");
        }

        double dx = LocalizerUtil.convertLonToMeters(1.0);
        double dy = LocalizerUtil.convertLatToMeters(1.5);
        if (Math.abs(GPSPoseMessage.getDistance(aPose, zeroPose) - Math.sqrt(dx * dx + dy * dy)) > .001) {
            fail("distance function does not handle some input correctly ");
        }


    }

    @Test
    public void testGotoWaypoint10meters() {
        LocalizerUtil.convertMetersToLat(10);

    }

}
