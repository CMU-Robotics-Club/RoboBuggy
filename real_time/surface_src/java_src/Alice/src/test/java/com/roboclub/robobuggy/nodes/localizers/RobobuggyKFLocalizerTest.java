import com.roboclub.robobuggy.messages.GPSPoseMessage;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import org.junit.Assert;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

import java.util.concurrent.LinkedBlockingQueue;

/**
 * Created by vivaanbahl on 11/16/16.
 */
public class RobobuggyKFLocalizerTest {

    private static LinkedBlockingQueue<GPSPoseMessage> poseMessages;

    @BeforeClass
    public static void oneTimeSetup() {
        new Subscriber("localizerTest", NodeChannel.POSE.getMsgPath(), ((topicName, m) -> {
            poseMessages.add(((GPSPoseMessage) m));
        }));
    }

    @Before
    public void setUp() {
        poseMessages = new LinkedBlockingQueue<>();
    }

    /**
     * NOTE THIS IS NOT YET A VALID TEST CASE
     * @throws InterruptedException
     */
    @Test
    public void test_singleIteration() throws InterruptedException {
        RobobuggyKFLocalizer localizer = new RobobuggyKFLocalizer(10000, "testLocalizer", new LocTuple(40.441670, -79.9416362));
        new Publisher(NodeChannel.GPS.getMsgPath()).publish(new GpsMeasurement(40.441670, -79.9512463));
        Thread.sleep(3000);
        localizer.update();
        Thread.sleep(3000);
        Assert.assertEquals(0.0, 1.0, 0.0);
    }

}