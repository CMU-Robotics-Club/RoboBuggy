package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.RobobuggyLogicNotificationMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

/**
 * The error console for any RobobuggyLogicNotifications
 */
public class LogicErrorMessageConsole extends RobobuggyGUIContainer {
    /**
     *
     */
    private static final long serialVersionUID = -6532100079403054035L;
    private static final RobobuggyMessageLevel[] ROBOBUGGY_MESSAGE_LEVELS = RobobuggyMessageLevel.values();
    private static final int NUMBER_OF_MESSAGES = ROBOBUGGY_MESSAGE_LEVELS.length;
    private boolean[] showMessages;
    private JButton[] buttons;
    private JScrollPane messageScrollingContainer;
    private JTextArea messages;

    private String[] messageBuffer = new String[10];
    private int messageBufferStart = 0;
    private int messageBufferEnd = 0;

    //buttons for showing which messages to show
    private JButton exceptionBtn;
    private JButton warningBtn;
    private JButton noteBtn;

    /**
     * The header bar for the console
     */
    public class Header extends RobobuggyGUIContainer {
        private static final long serialVersionUID = 8019501107506958218L;

        /**
         * Constructor for the Header
         */
        public Header() {
            showMessages = new boolean[NUMBER_OF_MESSAGES];
            buttons = new JButton[NUMBER_OF_MESSAGES];
            //init header buttons
            for (int k = 0; k < NUMBER_OF_MESSAGES; k++) {
                showMessages[k] = true;
                buttons[k] = new JButton(ROBOBUGGY_MESSAGE_LEVELS[k].toString());
            }


            JLabel label = new JLabel("Logic Errors");
            exceptionBtn = new JButton("SHOW EXCEPTION");
            exceptionBtn.addActionListener(new ActionListener() {

                @Override
                public void actionPerformed(ActionEvent e) {
                    showMessages[0] = !showMessages[0];
                    if (showMessages[0]) {
                        exceptionBtn.setBackground(Color.green);
                        exceptionBtn.setText("SHOW EXCEPTION");
                    } else {
                        exceptionBtn.setBackground(Color.gray);
                        exceptionBtn.setText("DONT SHOW EXCEPTION");

                    }
                }
            });
            warningBtn = new JButton("SHOW WARNING");
            warningBtn.addActionListener(new ActionListener() {

                @Override
                public void actionPerformed(ActionEvent e) {
                    showMessages[1] = !showMessages[1];
                    if (showMessages[1]) {
                        warningBtn.setBackground(Color.GREEN);
                        warningBtn.setText("SHOW WARNING");
                    } else {
                        warningBtn.setBackground(Color.GRAY);
                        warningBtn.setText("DONT SHOW WARNING");
                    }
                }
            });
            noteBtn = new JButton("SHOW NOTE");
            noteBtn.addActionListener(new ActionListener() {

                @Override
                public void actionPerformed(ActionEvent e) {
                    showMessages[2] = !showMessages[2];
                    if (showMessages[2]) {
                        noteBtn.setBackground(Color.GREEN);
                        noteBtn.setText("SHOW NOTE");
                    } else {
                        noteBtn.setBackground(Color.GRAY);
                        noteBtn.setText("DONT SHOW NOTE");

                    }


                }
            });
            this.addComponent(label, 0, 0, .25, 1);
            this.addComponent(exceptionBtn, .25, 0, .25, 1);
            this.addComponent(warningBtn, .50, 0, .25, 1);
            this.addComponent(noteBtn, .75, 0, .25, 1);


        }


    }

    /**
     * adds a RobobuggyLogicNotifications to the console list
     *
     * @param newMessage a new RobobuggyLogicNotifications as a string
     */
    public void addMessage(String newMessage) {
        messageBuffer[messageBufferEnd] = newMessage;
        String text = "";
        for (int i = 0; i < messageBuffer.length; i++) {
            text = text.concat(messageBuffer[(messageBufferStart + i) % messageBuffer.length] + "\n");
        }
        messageBufferEnd = (messageBufferEnd + 1) % messageBuffer.length;
        //move start up if need be
        if (messageBufferEnd == messageBufferStart) {
            messageBufferStart = (messageBufferStart + 1) % messageBuffer.length;
        }
        messages.setText(text);
        Gui.getInstance().fixPaint();

    }

    /**
     * Constructor for the logic exception console
     */
    public LogicErrorMessageConsole() {
        exceptionBtn = new JButton();
        warningBtn = new JButton();
        noteBtn = new JButton();

        Header header = new Header();
        exceptionBtn.setEnabled(true);
        warningBtn.setEnabled(true);
        noteBtn.setEnabled(true);
        messages = new JTextArea();
        messageScrollingContainer = new JScrollPane(messages);
        messageScrollingContainer.setVisible(true);
        this.addComponent(header, 0.0, 0.0, 1.0, .1);
        this.addComponent(messageScrollingContainer, 0.0, 0.1, 1.0, .9);

        messages.setText("this is where user defined messages will go\n");

        // Subscriber for LogicException updates
        new Subscriber("uiLogicErrorConsole", NodeChannel.LOGIC_NOTIFICATION.getMsgPath(), new MessageListener() {
            @Override
            public void actionPerformed(String topicName, Message m) {
                RobobuggyLogicNotificationMeasurement msg = (RobobuggyLogicNotificationMeasurement) m;
                switch (msg.getLevel()) {
                    case EXCEPTION:
                        if (showMessages[0]) {
                            addMessage(ROBOBUGGY_MESSAGE_LEVELS[0].toString() + ":\t" + msg.getMessage() + "\n");
                        }
                        break;
                    case WARNING:
                        if (showMessages[1]) {
                            addMessage(ROBOBUGGY_MESSAGE_LEVELS[1].toString() + "\t" + msg.getMessage() + "\n");
                        }
                        break;
                    case NOTE:
                        if (showMessages[2]) {
                            addMessage(ROBOBUGGY_MESSAGE_LEVELS[2].toString() + "\t" + msg.getMessage() + "\n");
                        }
                        break;
                    default:
                        addMessage("something is wrong with  the logic error console \n");
                }

                Gui.getInstance().fixPaint();

            }

        });


    }

}
