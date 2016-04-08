package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.messages.BrakeStateMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.ImageMessage;
import com.roboclub.robobuggy.messages.MagneticMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import com.roboclub.robobuggy.simulation.SensorPlayer;
import org.junit.Before;
import org.junit.Test;

import javax.imageio.ImageIO;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

/**
 * A class that checks to see if we can log and play back correctly
 */
public class FullLoggingTest {

    private Publisher gpsPub;
    private Publisher imuPub;
    private Publisher encoderPub;
    private Publisher steeringPub;
    private Publisher brakePub;
    private Publisher videoPub;

    private ArrayList<Message> masterList;
    private ArrayList<Message> simList;

    private boolean isPlaybackOccuring;

    private static final int LOG_FILE_LENGTH = 100;
    private static final int TIME_DELAY = 100;
    private static final boolean USE_DELAYS = true;

    /**
     * initializes all of the subscribers and publishers,
     */
    @Before
    public void oneTimeSetup() {

        gpsPub = new Publisher(NodeChannel.GPS.getMsgPath());
        imuPub = new Publisher(NodeChannel.IMU_MAGNETIC.getMsgPath());
        encoderPub = new Publisher(NodeChannel.ENCODER.getMsgPath());
        steeringPub = new Publisher(NodeChannel.STEERING.getMsgPath());
        brakePub = new Publisher(NodeChannel.BRAKE_STATE.getMsgPath());
        videoPub = new Publisher(NodeChannel.PUSHBAR_CAMERA.getMsgPath());

        MessageListener addToSimList = (topicName, m) -> {
            if (isPlaybackOccuring) {
                simList.add(m);
            }
        };

        new Subscriber("fullLoggingTest", NodeChannel.GPS.getMsgPath(), addToSimList);
        new Subscriber("fullLoggingTest", NodeChannel.IMU_MAGNETIC.getMsgPath(), addToSimList);
        new Subscriber("fullLoggingTest", NodeChannel.ENCODER.getMsgPath(), addToSimList);
        new Subscriber("fullLoggingTest", NodeChannel.STEERING.getMsgPath(), addToSimList);
        new Subscriber("fullLoggingTest", NodeChannel.BRAKE_STATE.getMsgPath(), addToSimList);
        new Subscriber("fullLoggingTest", NodeChannel.PUSHBAR_CAMERA.getMsgPath(), addToSimList);

    }

    /**
     * Cleans out the master and check list
     */
    @Before
    public void setup() {
        isPlaybackOccuring = false;
        masterList = new ArrayList<>();
        simList = new ArrayList<>();
    }

    /**
     * Adds random messages to a log, and then plays them back to see if they're in the same order
     * @throws IOException if we couldn't find an image for the message
     * @throws InterruptedException if java was a jerk and decided to interrupt us during our private testing time
     */
    @Test
    public void testLogFileSameAsSimResults() throws IOException, InterruptedException {
        LoggingNode ln = new LoggingNode(NodeChannel.GUI_LOGGING_BUTTON, "logs/", NodeChannel.getLoggingChannels());
        ln.startNode();
        Publisher loggingPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
        loggingPub.publish(
                new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.START)
        );

        Thread.sleep(100);
        File logFile = ln.getOutputFile();
        Random r = new Random();

        for (int i = 0; i < LOG_FILE_LENGTH; i++) {
            int randNum = r.nextInt(6);
            Message randMessage = null;

            switch (randNum) {
                case 0:
                    randMessage = new GpsMeasurement(Math.random(), Math.random());
                    gpsPub.publish(randMessage);
                    break;
                case 1:
                    randMessage = new MagneticMeasurement(Math.random(), Math.random(), Math.random());
                    imuPub.publish(randMessage);
                    break;
                case 2:
                    randMessage = new EncoderMeasurement(Math.random(), Math.random());
                    encoderPub.publish(randMessage);
                    break;
                case 3:
                    randMessage = new SteeringMeasurement(Math.random());
                    steeringPub.publish(randMessage);
                    break;
                case 4:
                    randMessage = new BrakeStateMessage(r.nextBoolean());
                    brakePub.publish(randMessage);
                    break;
                case 5:
                    randMessage = new ImageMessage(
                            ImageIO.read(new File("images/testImages/logImage" + r.nextInt(6) + ".png")), i
                    );
                    videoPub.publish(randMessage);
                    break;
                default:
                    break;
            }

            masterList.add(randMessage);
            if (USE_DELAYS) {
                Thread.sleep(TIME_DELAY);
            }
        }

        loggingPub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.STOP));

        // finished building the master list

        SensorPlayer player = new SensorPlayer(logFile.getPath(), 1);
        isPlaybackOccuring = true;
        player.start();
        player.join();

        // playback is finished, time to compare the lists
        assertEquals("What went into the log file didn't all come out!", masterList.size(), simList.size());

        for (int i = 0; i < masterList.size(); i++) {
            Message masterListMessage = masterList.get(i);
            Message simListMessage = simList.get(i);

            if (masterListMessage instanceof GpsMeasurement) {
                GpsMeasurement masterGPS = ((GpsMeasurement) masterListMessage);
                GpsMeasurement simGPS = ((GpsMeasurement) simListMessage);

                assertEquals(masterGPS.getLatitude(), simGPS.getLatitude(), 0);
                assertEquals(masterGPS.getLongitude(), simGPS.getLongitude(), 0);
            } else if (masterListMessage instanceof MagneticMeasurement) {
                MagneticMeasurement masterMag = ((MagneticMeasurement) masterListMessage);
                MagneticMeasurement simMag = ((MagneticMeasurement) simListMessage);

                assertEquals(masterMag.getmagX(), simMag.getmagX(), 0);
                assertEquals(masterMag.getmagY(), simMag.getmagY(), 0);
                assertEquals(masterMag.getmagZ(), simMag.getmagZ(), 0);
            } else if (masterListMessage instanceof EncoderMeasurement) {
                EncoderMeasurement masterEnc = ((EncoderMeasurement) masterListMessage);
                EncoderMeasurement simEnc = ((EncoderMeasurement) simListMessage);

                assertEquals(masterEnc.getAcceleration(), simEnc.getAcceleration(), 0);
                assertEquals(masterEnc.getDataWord(), simEnc.getDataWord(), 0);
                assertEquals(masterEnc.getDistance(), simEnc.getDistance(), 0);
                assertEquals(masterEnc.getVelocity(), simEnc.getVelocity(), 0);
            } else if (masterListMessage instanceof SteeringMeasurement) {
                SteeringMeasurement masterSteer = ((SteeringMeasurement) masterListMessage);
                SteeringMeasurement simSteer = ((SteeringMeasurement) simListMessage);

                assertEquals(masterSteer.getAngle(), simSteer.getAngle(), 0);
            } else if (masterListMessage instanceof BrakeStateMessage) {
                BrakeStateMessage masterBrake = ((BrakeStateMessage) masterListMessage);
                BrakeStateMessage simBrake = ((BrakeStateMessage) simListMessage);

                assert masterBrake.isDown() == simBrake.isDown();
            } else if (masterListMessage instanceof ImageMessage) {
                ImageMessage masterImage = ((ImageMessage) masterListMessage);
                ImageMessage simImage = ((ImageMessage) simListMessage);

                assertEquals(masterImage.getFrameNumber(), simImage.getFrameNumber(), 0);
            } else {
                fail("Found a message in the master list we didn't create!");
            }


        }
    }

}
