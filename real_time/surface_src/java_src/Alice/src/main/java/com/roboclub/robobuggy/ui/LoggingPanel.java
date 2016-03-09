package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Publisher;

import javax.swing.JButton;
import javax.swing.JFormattedTextField;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.Timer;
import java.awt.Color;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * {@link RobobuggyGUIContainer} used for logging information
 */
public class LoggingPanel extends RobobuggyGUIContainer{
	//TODO: test to make sure all these dont need to be static
	private JButton resetBtn;
	private JButton playBtn;
	private JFormattedTextField timeLbl;
	private static final int TIME_ZONE_OFFSET = 18000000;//5 hours
	private Date startTime;
	private Timer timer;
	private Publisher loggingButtonPub;
	//to make logging panel accessible to button callbacks
	private LoggingPanel thisLoggingPanel = this;
	private JLabel filenameLabel;
	private JTextField playbackSpeed;

	/**
	 * Construct a new {@link LoggingPanel} object
	 */
	public LoggingPanel(){
		name = "LoggingPanel";
		loggingButtonPub = new Publisher(NodeChannel.GUI_LOGGING_BUTTON.getMsgPath());

		timer = new Timer(10, new TimerHandler());// updates every .01 seconds
		timer.setDelay(100);
		timer.setRepeats(true); // timer needs to be setup before startpause_btn

		//should be the time that we start the sytem at 
		startTime = new Date();
		
		playBtn = new JButton("START");
		playBtn.setFont(new Font("serif", Font.PLAIN, 70));
		playBtn.addActionListener(new PlayButtonHandler());
		playBtn.setEnabled(true);
		playBtn.setBackground(Color.BLUE);
		
		resetBtn = new JButton("Reset encoder");
		resetBtn.setFont(new Font("seif", Font.PLAIN, 50));
		resetBtn.addActionListener(new ResetButtonHandler());
		resetBtn.setEnabled(true);
		resetBtn.setBackground(Color.BLUE);
	
		filenameLabel = new JLabel("File: ",
				SwingConstants.CENTER);
		filenameLabel.setFont(new Font("sanserif", Font.PLAIN, 15));

		timeLbl = new JFormattedTextField(new SimpleDateFormat("HH:mm:ss.S"));
		timeLbl.setHorizontalAlignment(SwingConstants.CENTER);
		timeLbl.setFont(new Font("sanserif", Font.PLAIN, 50));
		timeLbl.setEditable(false);
		timeLbl.setColumns(7);
		timeLbl.setValue(startTime);

		playbackSpeed = new JTextField("1");
		playbackSpeed.setHorizontalAlignment(JTextField.CENTER);

		this.addComponent(playBtn, 0, 0, 1.0, .25);
		this.addComponent(filenameLabel, 0, .25, 0.5, .25);
		this.addComponent(timeLbl, 0, .5, 1, .5);
		this.addComponent(playbackSpeed, .5, .25, 0.5, .25);
	}


	/**
	 * Sets the label for which file we are currently logging to
	 * @param fileName the filepath we're logging to
	 */
	public void setFileName(String fileName) {
		filenameLabel.setText("File: " + fileName);
	}

	/**
	 * safe returns the speed
	 * @return the speed, or 1 if the text in the field can't be serialized
	 */
	public double getPlaybackSpeed() {
		String speedStr = playbackSpeed.getText();
		if (speedStr.equals("")) return 1;

		try {
			return Double.valueOf(speedStr);
		}
		catch (NumberFormatException e) {
			return 1;
		}
	}
	
	private class ResetButtonHandler implements ActionListener{
		public void actionPerformed(ActionEvent e){
			
		}
	}

	/**
	 * Class that implements what the clicking the start and stop button should d
	 */
	private class PlayButtonHandler implements ActionListener {
		private boolean isLogging = false;
		
		@Override
		public void actionPerformed(ActionEvent e) {
			// inverts the state of the system every time the button is pressed
			new RobobuggyLogicNotification("start/stop button was pressed", RobobuggyMessageLevel.NOTE);
		
			if (!isLogging) {
                isLogging = true;
				enableLogging();
			} else {
                isLogging = false;
				disableLogging();
            }//end if else
            thisLoggingPanel.validate();
            Gui.getInstance().fixPaint();  //if this line is not here then the gui will not display correctly after the button is pressed

        }
    }

	/**
	 * This class handles timers, what that does is still a mystery....
	 */
    private class TimerHandler implements ActionListener {
        @Override
        public void actionPerformed(ActionEvent ae) {
            Date currentTime = new Date();
            timeLbl.setValue(currentTime.getTime() - startTime.getTime()+ TIME_ZONE_OFFSET);
            Gui.getInstance().fixPaint();
        }
    }

	/*	Update Methods */
    /**
     * Enable logging
     */
    public void enableLogging() {
        timer.start();
        playBtn.setBackground(Color.RED);
        playBtn.setText("STOP");

        loggingButtonPub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.START));
        startTime = new Date();
    }

    /**
     * Disable logging
     */
    public void disableLogging() {
        playBtn.setBackground(Color.GREEN);
        playBtn.setText("START");

        loggingButtonPub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.STOP));
        timer.stop();
    }

}