package com.roboclub.robobuggy.main;

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
    private Date startPressedTime;	
    private Timer timer;
    DateFormat df = new SimpleDateFormat("HH:mm:ss.S");
    
	public ControlsPanel() {
		//stuff for setting up logging ie start/stop, file name ...
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridLayout(2, 1));
		JPanel top_panel = new JPanel();
		top_panel.setLayout(new GridLayout(1,2));
		startPause_btn = new JButton("Start");
		startPause_btn.setFont(new Font("serif", Font.PLAIN, 70));
		updateStartPause_btn();
		StartPauseButtonHandler startPauseHandler = new StartPauseButtonHandler();
		startPause_btn.addActionListener(startPauseHandler);
		JLabel currentFile_lbl = new JLabel("currentFile",SwingConstants.CENTER);
		JLabel newFile_lbl = new JLabel("newFile",SwingConstants.CENTER);
		

		time_lbl = new JLabel("",SwingConstants.CENTER);
		time_lbl.setFont(new Font("sanserif",Font.PLAIN,70));
		
		timer = new Timer(10, new timerHandler());//updates every .01 seconds
		timer.setDelay(100);
	    timer.setRepeats(true);	
	   

	    top_panel.add(startPause_btn);
	    top_panel.add(currentFile_lbl);
	    top_panel.add(newFile_lbl);
	    top_panel.add(time_lbl);
	    this.add(top_panel);
		
	    JPanel bottom_panel = new JPanel();
	    bottom_panel.setLayout(new GridLayout(1, 2));
	    
	    SensorSwitchPanel gps_switch = new SensorSwitchPanel("GPS",Sensor_state_type.ON);
	    SensorSwitchPanel frontCam_switch = new SensorSwitchPanel("Front Cam",Sensor_state_type.ON);
	    SensorSwitchPanel backCam_switch = new SensorSwitchPanel("Back Cam",Sensor_state_type.ON);
	    SensorSwitchPanel encoders_switch = new SensorSwitchPanel("Encoders",Sensor_state_type.ON);
	    SensorSwitchPanel IMU_switch = new SensorSwitchPanel("IMU",Sensor_state_type.ON);
	    SensorSwitchPanel controlInputs_switch = new SensorSwitchPanel("Control Inputs",Sensor_state_type.ON);
	    SensorSwitchPanel logging_switch = new SensorSwitchPanel("Logging",Sensor_state_type.ON);
	    SensorSwitchPanel autonomous_switch = new SensorSwitchPanel("Autonomous",Sensor_state_type.OFF);
 
	    
	    bottom_panel.add(gps_switch.getGraphics());
	    bottom_panel.add(frontCam_switch.getGraphics());
	    bottom_panel.add(backCam_switch.getGraphics());
	    bottom_panel.add(encoders_switch.getGraphics());
	    bottom_panel.add(IMU_switch.getGraphics());
	    bottom_panel.add(controlInputs_switch.getGraphics());
	    bottom_panel.add(logging_switch.getGraphics());
	    bottom_panel.add(autonomous_switch.getGraphics());

	    this.add(bottom_panel);
		
		
	}
	
	//updates the display based on external events
	public void updatePanel(){
		//TODO
		updateStartPause_btn();
	}
	
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
	
	private void updateStartPause_btn(){
		if(Gui.getInstance().GetPlayPauseState())
		{	
			System.out.println("System Started");
			startPause_btn.setBackground(Color.RED);
			startPause_btn.setText("Pause");			
		} else {
			System.out.println("System Paused");
			startPause_btn.setBackground(Color.GREEN);
		}
		repaint();		
		
	}
	
	
	private class StartPauseButtonHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			//inverts the state of the system every time the button is pressed
			if(Gui.getInstance().GetPlayPauseState()){
				Gui.setPlayPauseState(false);
				timer.start();
			    startPressedTime = new Date();
			}else{
				Gui.setPlayPauseState(true);
				timer.stop();
			}
			updateStartPause_btn();

		}
	}

}
