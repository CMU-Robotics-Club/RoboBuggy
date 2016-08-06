package com.roboclub.robobuggy.nodes.sensors;

import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import junit.framework.TestCase;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.runners.model.TestClass;

import java.io.*;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Created by vivaanbahl on 7/27/16.
 */
public class LoggingNodeTest extends TestCase {

    private LoggingNode logger;
    private Publisher triggerPub;

    @Before
    public void setUp() throws Exception {
        triggerPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());

        logger = new LoggingNode("./", NodeChannel.getLoggingChannels());
    }

    @After
    public void tearDown() throws Exception {

    }

    private void startLogging() {
        triggerPub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.START));
    }

    private void stopLogging() throws InterruptedException {
        triggerPub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.STOP));
        // allow for settle time
        Thread.sleep(1000);
    }

    private String getLogString() throws IOException {
        File outputFile = logger.outputFile;
        BufferedReader reader = new BufferedReader(new FileReader(outputFile));
        String file = "";
        String line;
        while ((line = reader.readLine()) != null) {
            file += line + "\n";
        }

        return file;
    }

    private void compareFileToMessageList(String messageList) throws IOException {
        String fileRegex =
                "\\{\"name\":\"Robobuggy Data Logs\",\"schema_version\":\"\\d+.0\",\"date_recorded\":\"\\d+-\\d+-\\d+ " +
                        "\\d+:\\d+:\\d+.\\d+\",\"software_version\":\"\\w+\"\\}\n" +
                "\\{\"VERSION_ID\":\"logic_notification\",\"message\":\"Starting up logging thread!\",\"level\":" +
                        "\"NOTE\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm:ss.SSS\",\"timestamp\":\\d+,\"topicName\"" +
                        ":\"sensors\\/logic_notification\",\"sequenceNumber\":\\d+\\},\n" +
                "\\{\"VERSION_ID\":\"gui_logging_buttonV0.0\",\"lm\":\"STOP\",\"DATE_FORMAT\":\"yyyy-MM-dd HH:mm" +
                        ":ss.SSS\",\"timestamp\":\\d+,\"topicName\":\"sensors\\/logging_button\",\"sequenceNumber" +
                        "\":\\d+\\},\n" +
                messageList +
                "\\{\"VERSION_ID\":\"STOP\"\\}\n" +
                "\\{\"VERSION_ID\":\"data_breakdown\",\"logging_button\":\\d+,\"gps\":0,\"imu\":0,\"encoder\":0," +
                        "\"brake_state\":0,\"steering\":0,\"fp_hash\":0,\"logic_notification\":1,\"push_bar_camera" +
                        "\":0,\"pose\":0,\"reset\":0,\"state\":0\\}";

        String logFile = getLogString();
        Pattern filePattern = Pattern.compile(fileRegex);
        Matcher fileMatcher = filePattern.matcher(Pattern.quote(logFile));

        if (!fileMatcher.find()) {
            fail("File didn't match the provided regex!");
        }

    }

    /**
     * Logs a file without any incoming messages
     */
    public void testLogEmptyFile() throws InterruptedException, IOException {
        startLogging();
        Thread.sleep(3000);
        stopLogging();

        compareFileToMessageList("");
    }

    /**
     * Logs a file with a single publisher publishing messages
     */
    public void testOnePub() {
        fail("Not implemented");
    }

    /**
     * Logs a file with multiple publishers
     */
    public void testMultiPub() {
        fail("Not implemented");
    }

}