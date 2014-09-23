package com.roboclub.robobuggy.main;

import java.awt.Color;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.Timer;

public class ControlsPanel extends JPanel {
	private static final long serialVersionUID = -924045896215455343L;
	
	// Big gui objects
	private JButton startPause_btn;
    private Date startPressedTime;	
    private Timer timer;
    private JLabel time_lbl;
    private JTextField filename;
    private JCheckBox displayFeed;
    
	public ControlsPanel() {
		//stuff for setting up logging ie start/stop, file name ...
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout( new FlowLayout(FlowLayout.CENTER));
		
		startPause_btn = new JButton("Start");
		startPause_btn.setFont(new Font("serif", Font.PLAIN, 70));
		//TODO move following into a function 
		if(Gui.GetPlayPauseState())
		{	
			startPause_btn.setBackground(Color.RED);
			startPause_btn.setText("Pause");
			startPressedTime = new Date();
		} else {
			startPause_btn.setBackground(Color.GREEN);
			startPause_btn.setText("Start");
		}		
		
		StartPauseButtonHandler startPauseHandler = new StartPauseButtonHandler();
		startPause_btn.addActionListener(startPauseHandler);
		JLabel currentFile_lbl = new JLabel("currentFile",SwingConstants.CENTER);
		JLabel newFile_lbl = new JLabel("newFile",SwingConstants.CENTER);
		

		time_lbl = new JLabel("",SwingConstants.CENTER);
		time_lbl.setFont(new Font("sanserif",Font.PLAIN,70));
		
		timer = new Timer(10, new timerHandler());//updates every .01 seconds
		timer.setDelay(100);
	    timer.setRepeats(true);
	    
	    displayFeed = new JCheckBox();
	    displayFeed.setEnabled(true);
	    displayFeed.setSelected(Gui.GetDisplayState());
	    displayFeed.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				Gui.setDisplayState(displayFeed.isSelected());
			}
	    });
	    
		this.add(startPause_btn);
		this.add(currentFile_lbl);
		this.add(newFile_lbl);
		this.add(time_lbl);
		this.add(displayFeed);
	}
	
	DateFormat df = new SimpleDateFormat("HH:mm:ss.S");
	private class timerHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent ae) {
			Date now = new Date();
			long difference = now.getTime() - startPressedTime.getTime();
			time_lbl.setText(df.format(now) + "/" + df.format(new Date(difference)));
			
			repaint();
		}
	}
	private class StartPauseButtonHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			Gui.getInstance();
			//inverts the state of the system every time the button is pressed 
			Gui.setPlayPauseState(!Gui.GetPlayPauseState());
			if(Gui.GetPlayPauseState())
			{	
				System.out.println("System Started");
				startPause_btn.setBackground(Color.RED);
				startPause_btn.setText("Pause");
				
				timer.start();
				startPressedTime = new Date();
			} else {
				System.out.println("System Paused");
				startPause_btn.setBackground(Color.GREEN);
				startPause_btn.setText("Start");
				timer.stop();
			}
			repaint();
		}
	}

}

