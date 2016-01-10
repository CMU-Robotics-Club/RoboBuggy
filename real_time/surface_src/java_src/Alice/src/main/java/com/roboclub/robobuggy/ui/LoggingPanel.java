package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JFormattedTextField;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.Timer;

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.main.RobobuggyLogicException;
import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.messages.GuiLoggingButtonMessage;
import com.roboclub.robobuggy.ros.Publisher;

public class LoggingPanel extends RoboBuggyGUIContainer{
	private static JButton play_btn;
	private JFormattedTextField time_lbl;
	private static final int TIME_ZONE_OFFSET = 18000000;//5 hours
	private static Date startTime;
	private static Timer timer;
	Publisher logging_button_pub;
	//to make logging panel accessible to button callbacks
	private LoggingPanel thisLoggingPanel = this;
	

	public  LoggingPanel(){
		name = "LoggingPanel";
		logging_button_pub = new Publisher(Gui.GuiPubSubTopics.GUI_LOG_BUTTON_UPDATED.toString());
		
		timer = new Timer(10, new timerHandler());// updates every .01 seconds
		timer.setDelay(100);
		timer.setRepeats(true); // timer needs to be setup before startpause_btn

		//should be the time that we start the sytem at 
		startTime = new Date();
		
	play_btn = new JButton("START");
	play_btn.setFont(new Font("serif", Font.PLAIN, 70));
	play_btn.addActionListener(new PlayButtonHandler());
	play_btn.setEnabled(true);
	play_btn.setBackground(Color.BLUE);
	
	JLabel filename_lbl = new JLabel("File: ",
			SwingConstants.CENTER);
	filename_lbl.setFont(new Font("sanserif", Font.PLAIN, 30));

	time_lbl = new JFormattedTextField(new SimpleDateFormat("HH:mm:ss.S"));
	time_lbl.setHorizontalAlignment(SwingConstants.CENTER);
	time_lbl.setFont(new Font("sanserif", Font.PLAIN, 50));
	time_lbl.setEditable(false);
	time_lbl.setColumns(7);
	time_lbl.setValue(startTime);

	this.addComponet(play_btn, 0, 0, 1.0, .25);
	this.addComponet(filename_lbl, 0, .25, 1.0, .25);
	this.addComponet(time_lbl, 0, .5, 1, .5);
	}

private class PlayButtonHandler implements ActionListener {
	@Override
	public void actionPerformed(ActionEvent e) {
		// inverts the state of the system every time the button is pressed
		config.active = !config.active;
		new RobobuggyLogicException("start/stop button was pressed", MessageLevel.NOTE);
		
		if (config.active) {
			System.out.println("System Started");
			play_btn.setBackground(Color.RED);
			play_btn.setText("STOP");
			timer.start();
			
			RobotLogger.CreateLog();
			logging_button_pub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.START));
			startTime = new Date();
		} else {
			System.out.println("System Paused");
			play_btn.setBackground(Color.GREEN);
			play_btn.setText("START");
			
			RobotLogger.CloseLog();
			logging_button_pub.publish(new GuiLoggingButtonMessage(GuiLoggingButtonMessage.LoggingMessage.STOP));
			timer.stop();
		}//end if else
		thisLoggingPanel.validate();
		Gui.getInstance().fixPaint();  //if this line is not here then the gui will not display correctly after the button is pressed

	}
}

private class timerHandler implements ActionListener {
	@Override
	public void actionPerformed(ActionEvent ae) {
		Date currentTime = new Date();
		time_lbl.setValue(currentTime.getTime() - startTime.getTime()+ TIME_ZONE_OFFSET);
	    Gui.getInstance().fixPaint();
	}
}

/*		Update Methods */
public void enableLogging() {
	play_btn.setEnabled(true);
	play_btn.setBackground(Color.GREEN);
}
}