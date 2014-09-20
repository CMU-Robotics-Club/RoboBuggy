package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.Timer;

public class ControlsPanel extends JPanel {
	private static final long serialVersionUID = -924045896215455343L;
	
	// Big gui objects
	private JButton startPause_btn;
	private JLabel time_lbl;
	private boolean playPauseState;
	
	public ControlsPanel() {
		//stuff for setting up logging ie start/stop, file name ...
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		JPanel dataLoggingPanel = new JPanel();
		dataLoggingPanel.setLayout(new GridLayout(4, 1));
		startPause_btn = new JButton("Start");
		playPauseState = true;
		startPause_btn.setFont(new Font("serif", Font.PLAIN, 70));
		startPause_btn.setBackground(Color.GREEN);
		StartPauseButtonHandler startPauseHandler = new StartPauseButtonHandler();
		startPause_btn.addActionListener(startPauseHandler);
		JLabel currentFile_lbl = new JLabel("currentFile",SwingConstants.CENTER);
		JLabel newFile_lbl = new JLabel("newFile",SwingConstants.CENTER);
		Date dateobj = new Date();
		
	    System.out.println("");

		time_lbl = new JLabel("SystemTime: " + df.format(dateobj) + " logTime: ",SwingConstants.CENTER);
	
		Timer timer = new Timer(10, new timerHandler());//updates every .01 seconds
		timer.setInitialDelay(200); //waits .2 seconds to start for first time
		timer.setDelay(200);
	    timer.setRepeats(true);	
	    timer.setCoalesce(false);
	    timer.start();

		dataLoggingPanel.add(startPause_btn);
		dataLoggingPanel.add(currentFile_lbl);
		dataLoggingPanel.add(newFile_lbl);
		dataLoggingPanel.add(time_lbl);
	}
	
	DateFormat df = new SimpleDateFormat("dd/MM/yy HH:mm:ss");
	private class timerHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent ae) {
			Date now = new Date();
			time_lbl.setText("REPAINTED SystemTime: " + df.format(now) + " logTime: ");
			repaint();
		}
	}
	private class StartPauseButtonHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			
			System.out.println("Button was pressed!");
			//inverts the state of the system every time the button is pressed 
			playPauseState = !playPauseState;
			if(playPauseState)
			{	
				startPause_btn.setBackground(Color.RED);
				startPause_btn.setText("Pause");
			}else
			{	
				startPause_btn.setBackground(Color.GREEN);
				startPause_btn.setText("Start");
			}
			repaint();
		}
	}

}

