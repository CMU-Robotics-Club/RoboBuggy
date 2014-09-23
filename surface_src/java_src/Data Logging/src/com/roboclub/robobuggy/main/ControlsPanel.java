package com.roboclub.robobuggy.main;

import java.awt.BorderLayout;
import java.awt.Color;
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

import com.roboclub.robobuggy.logging.RobotLogger;

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
		// TODO add layout manager
		
		setupStartPauseBtn();
		
		//TODO fix displaying clock
		time_lbl = new JLabel("",SwingConstants.CENTER);
		time_lbl.setFont(new Font("sanserif",Font.PLAIN,70));
		
		timer = new Timer(10, new timerHandler());//updates every .01 seconds
		timer.setDelay(100);
	    timer.setRepeats(true);
	    
	    JLabel file_lbl = new JLabel("Filename: ", SwingConstants.CENTER);
	    file_lbl.setFont(new Font("sanserif",Font.PLAIN,20));
	    
	    //TODO set filename box to fixed size
	    filename = new JTextField(RobotLogger.getFilename(), 
	    		SwingConstants.CENTER);
	    filename.setSize(200, 50);
	    filename.setFont(new Font("sanserif",Font.PLAIN,20));
	    filename.setEditable(false);

	    JLabel display_lbl = new JLabel("Display Cameras: ", SwingConstants.CENTER);
	    display_lbl.setFont(new Font("sanserif",Font.PLAIN,20));
	    
	    displayFeed = new JCheckBox();
	    // TODO increase size of checkbox
	    displayFeed.setEnabled(true);
	    displayFeed.setSelected(Gui.GetDisplayState());
	    displayFeed.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				Gui.setDisplayState(displayFeed.isSelected());
			}
	    });
	    
		this.add(startPause_btn);
		this.add(time_lbl);
		this.add(file_lbl);
		this.add(filename);
		this.add(display_lbl);
		this.add(displayFeed);
	}
	
	private void setupStartPauseBtn() {
		startPause_btn = new JButton("Start");
		startPause_btn.setFont(new Font("serif", Font.PLAIN, 70));
		
		if(Gui.GetPlayPauseState())
		{	
			startPause_btn.setBackground(Color.RED);
			startPause_btn.setText("Pause");
			startPressedTime = new Date();
		} else {
			startPause_btn.setBackground(Color.GREEN);
			startPause_btn.setText("Start");
		}
		
		startPause_btn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
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
		});
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

}

