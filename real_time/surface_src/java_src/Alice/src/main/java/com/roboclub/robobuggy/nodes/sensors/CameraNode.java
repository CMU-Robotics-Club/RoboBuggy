package com.roboclub.robobuggy.nodes.sensors;

import com.github.sarxos.webcam.Webcam;
import com.github.sarxos.webcam.WebcamResolution;
import com.google.gson.JsonObject;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.ImageMessage;
import com.roboclub.robobuggy.messages.NodeStatusMessage;
import com.roboclub.robobuggy.nodes.baseNodes.BuggyBaseNode;
import com.roboclub.robobuggy.nodes.baseNodes.PeriodicNode;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.Subscriber;
import org.jcodec.api.awt.SequenceEncoder;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.List;

/**
 * @author Trevor Decker
 *         <p>
 *         A software driver for listening to a camera and publishing images from
 *         it to a message channel
 */
public class CameraNode extends PeriodicNode {
    private Webcam webcam;
    private Publisher imagePublisher;
    private int count = 0;
    private SequenceEncoder videoEncoder;

    static {
        Webcam.setHandleTermSignal(true);
    }

    /**
     * @param channel The channel to publish messages on
     * @param period  How often new images should be pulled
     */
    public CameraNode(NodeChannel channel, int period) {
        super(new BuggyBaseNode(channel), period, "Camera_Node");


        //setup the webcam
        List<Webcam> webcams = Webcam.getWebcams();

        //TODO figure out a better way to select
        for (Webcam webcam : webcams) {
            if (webcam.getName().contains("Logitech")) {
                this.webcam = webcam;
                this.webcam.setViewSize(WebcamResolution.QVGA.getSize());
                this.webcam.open();
                break;
            }
        }

        if (this.webcam == null) {
            new RobobuggyLogicNotification("Couldn't find Logitech webcam!", RobobuggyMessageLevel.EXCEPTION);
            this.webcam = Webcam.getDefault();
            // your camera have to support HD720p to run this code
            Webcam webcam = Webcam.getDefault();
            webcam.setCustomViewSizes(new Dimension[] { WebcamResolution.QVGA.getSize(), WebcamResolution.HD720.getSize() });
            webcam.setViewSize(WebcamResolution.QVGA.getSize());

        }

        //setup image publisher
        imagePublisher = new Publisher(channel.getMsgPath());

        setupLoggingTrigger();
        resume();
    }

    private void setupLoggingTrigger() {

        new Subscriber("cam", NodeChannel.NODE_STATUS.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                try {
                    NodeStatusMessage message = (NodeStatusMessage) m;
                    INodeStatus status = message.getMessage();

                    if (status.equals(LoggingNode.LoggingNodeStatus.STARTED_LOGGING)) {
                        JsonObject params = message.getParams();
                        String outputDir = params.get("outputDir").getAsString();

                        if (!webcam.isOpen()) {
                            webcam.open();
                        }
                        videoEncoder = new SequenceEncoder(new File(outputDir + "/webcam.mp4"));
                        new RobobuggyLogicNotification("Camera ready", RobobuggyMessageLevel.NOTE);
                    } else if (status.equals(LoggingNode.LoggingNodeStatus.STOPPED_LOGGING)) {
                        if (webcam.isOpen()) {
                            videoEncoder.finish();
                            webcam.close();
                        }

                    } else {
                        new RobobuggyLogicNotification("Status not recognized by CameraNode", RobobuggyMessageLevel.WARNING);
                    }
                } catch (IOException e) {
                    new RobobuggyLogicNotification("Log directory doesn't exist!", RobobuggyMessageLevel.EXCEPTION);
                }
            }
        });

    }


    @Override
    protected boolean startDecoratorNode() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    protected boolean shutdownDecoratorNode() {
        return true;
    }

    @Override
    protected void update() {
        try {
            if (webcam != null && imagePublisher != null) {
                if (webcam.isOpen()) {
                    BufferedImage mostRecentImage = webcam.getImage();
                    imagePublisher.publish(new ImageMessage(mostRecentImage, count));
                    count = count + 1;

                    if (videoEncoder != null) {
                        videoEncoder.encodeImage(mostRecentImage);
                    }
                }
            }
        } catch (IOException e) {
            new RobobuggyLogicNotification("Something went wrong trying to get image!", RobobuggyMessageLevel.EXCEPTION);
        }

    }

}
