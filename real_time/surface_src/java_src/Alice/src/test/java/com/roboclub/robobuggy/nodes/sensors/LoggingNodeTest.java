package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.messages.BatteryLevelMessage;
import com.roboclub.robobuggy.messages.BrakeMessage;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import org.junit.Assert;
import org.junit.BeforeClass;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Created by vivaanbahl on 7/27/16.
 */
public class LoggingNodeTest {

    private static LoggingNode logger;
    private static Publisher triggerPub;

    @BeforeClass
    public static void oneTimeSetup() {
        logger = new LoggingNode("./", NodeChannel.getLoggingChannels());
        triggerPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());
    }

    private void startLogging() throws InterruptedException {
        triggerPub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.START));
        // allow for setup time
        Thread.sleep(1000);
    }

    private void stopLogging() throws InterruptedException {
        // allow for settle time
        Thread.sleep(1000);
        triggerPub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.STOP));
        // allow for settle time
        Thread.sleep(1000);
    }

    private String getLogString() throws IOException {
        File outputFile = logger.outputFile;
        Scanner reader = new Scanner(outputFile, "UTF-8");
        String file = "";
        String line;
        while (reader.hasNextLine()) {
            line = reader.nextLine();
            file = file.concat(line);
            file = file.concat("\n");
        }
        reader.close();

        return file;
    }

    private void compareFileToMessageList(String messageList) throws IOException {
        String fileRegex =
                "\\{\"name\":\"Robobuggy Data Logs\",\"schema_version\":\\d+.\\d+,\"date_recorded\":\"\\d+-\\d+-\\d+ "+
                        "\\d+:\\d+:\\d+.\\d+\",\"software_version\":\".+\"\\}\n" +
                "\\{\"VERSION_ID\":\"logic_notification\",\"message\":\"Starting up logging thread!\",\"level\":" +
                        "\"NOTE\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\",\"timestamp\":\\d+,\"topicName\"" +
                        ":\"sensors\\/logic_notification\",\"sequenceNumber\":\\d+\\}\n" +
                messageList +
                "\\{\"VERSION_ID\":\"logic_notification\",\"message\":\"Stopping logging thread!\",\"level\":\"NOTE\"" +
                        ",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\",\"timestamp\":\\d+,\"topicName\":" +
                        "\"sensors\\/logic_notification\",\"sequenceNumber\":\\d+\\}\n" +
                "\\{\"VERSION_ID\":\"STOP\"\\}\n" +
                "\\{\"VERSION_ID\":\"data_breakdown\",\"logging_button\":\\d+,\"gps\":\\d+,\"imu\":\\d+,\"encoder\":" +
                        "\\d+,\"brake_state\":\\d+,\"steering\":\\d+,\"fp_hash\":\\d+,\"logic_notification\":\\d+," +
                        "\"push_bar_camera\":\\d+,\"pose\":\\d+,\"reset\":\\d+,\"state\":\\d+\\}";

        String logFile = getLogString();
        Pattern filePattern = Pattern.compile(fileRegex);
        Matcher fileMatcher = filePattern.matcher(Pattern.quote(logFile));

        if (!fileMatcher.find()) {
            Assert.fail("Couldn't match regex");
            System.out.println(logFile);
        }

    }

    /**
     * Logs a file without any incoming messages
     */
    @Test
    public void testLogEmptyFile() throws InterruptedException, IOException {
        startLogging();
        stopLogging();

        compareFileToMessageList("");
        Assert.assertTrue(true);
    }

    /**
     * Logs a file with a single publisher publishing messages
     */
    @Test
    public void testOnePub() throws InterruptedException, IOException {
        Publisher batteryPub = new Publisher(NodeChannel.BATTERY.getMsgPath());
        startLogging();

        for (int i = 0; i < 10; i++) {
            batteryPub.publish(new BatteryLevelMessage(1337));
            Thread.sleep(500);
        }

        stopLogging();

        String extraMessages = "";
        String battMessage = "\\{\"VERSION_ID\":\"BatteryV0.1\",\"level\":1337,\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm" +
                        ":ss.SSS\",\"timestamp\":\\d+,\"topicName\":\"sensors\\/battery\",\"sequenceNumber" +
                        "\":\\d+\\}\n";
        for (int i = 0; i < 10; i++) {
            extraMessages = extraMessages.concat(battMessage);
        }

        compareFileToMessageList(extraMessages);
        Assert.assertTrue(true);
    }

    /**
     * Logs a file with multiple publishers
     */
    @Test
    public void testMultiPub() throws InterruptedException, IOException {
        Publisher batteryPub = new Publisher(NodeChannel.BATTERY.getMsgPath());
        Publisher brakePub = new Publisher(NodeChannel.BRAKE_CTRL.getMsgPath());
        Publisher steeringPub = new Publisher(NodeChannel.STEERING.getMsgPath());
        startLogging();

        for (int i = 0; i < 10; i++) {
            batteryPub.publish(new BatteryLevelMessage(1337));
            Thread.sleep(500);
            brakePub.publish(new BrakeMessage(0));
            Thread.sleep(500);
            steeringPub.publish(new SteeringMeasurement(1337.0));
            Thread.sleep(500);
        }

        stopLogging();

        String extraMessages = "";
        String battMessage = "\\{\"VERSION_ID\":\"BatteryV0.1\",\"level\":1337,\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm" +
                ":ss.SSS\",\"timestamp\":\\d+,\"topicName\":\"sensors\\/battery\",\"sequenceNumber" +
                "\":\\d+\\}\n";
        String brakeMessage = "\\{\"VERSION_ID\":\"brakeV0.1\",\"down\":false,\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm" +
                ":ss.SSS\",\"timestamp\":\\d+,\"topicName\":\"sensors\\/commanded brake\",\"sequenceNumber" +
                "\":\\d+\\}\n";
        String steeringMessage = "\\{\"VERSION_ID\":\"steering\",\"angle\":1337.0,\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm" +
                ":ss.SSS\",\"timestamp\":\\d+,\"topicName\":\"sensors\\/steering\",\"sequenceNumber" +
                "\":\\d+\\}\n";

        for (int i = 0; i < 10; i++) {
            extraMessages = extraMessages.concat(battMessage);
            extraMessages = extraMessages.concat(brakeMessage);
            extraMessages = extraMessages.concat(steeringMessage);
        }

        compareFileToMessageList(extraMessages);
        Assert.assertTrue(true);
    }

}