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

import com.roboclub.robobuggy.logging.RobotLogger;
import com.roboclub.robobuggy.main.Robot;
import com.roboclub.robobuggy.main.config;
import com.roboclub.robobuggy.sensors.SensorState;

/**
 * 
 * @author Trevor Decker 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class ControlPanel extends JPanel {
	private static final long serialVersionUID = -924045896215455343L;
	
	// Big gui objects
	private static JButton startPause_btn;
	private JLabel time_lbl;
    private static Date startPressedTime;	
    private static Timer timer;
    private static Timer updateTimer;

    DateFormat df = new SimpleDateFormat("HH:mm:ss.S");
    
    SensorSwitchPanel gps_switch;
    SensorSwitchPanel frontCam_switch;
    SensorSwitchPanel backCam_switch; 
    SensorSwitchPanel encoders_switch; 
    SensorSwitchPanel IMU_switch; 
    SensorSwitchPanel controlInputs_switch;
    SensorSwitchPanel logging_switch;
    SensorSwitchPanel autonomous_switch;
    
	public ControlPanel() {
		timer = new Timer(10, new timerHandler());//updates every .01 seconds
		timer.setDelay(100);
	    timer.setRepeats(true);	//timer needs to be setup before startpause_btn
	    
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
		

	   

	    top_panel.add(startPause_btn);
	    top_panel.add(currentFile_lbl);
	    top_panel.add(newFile_lbl);
	    top_panel.add(time_lbl);
	    this.add(top_panel);
		
	    JPanel bottom_panel = new JPanel();
	    bottom_panel.setLayout(new GridLayout(1, 2));
	    
	     gps_switch = new SensorSwitchPanel("GPS",SensorState.DISCONECTED);
	     frontCam_switch = new SensorSwitchPanel("Front Cam",SensorState.DISCONECTED);
	     backCam_switch = new SensorSwitchPanel("Back Cam",SensorState.DISCONECTED);
	     encoders_switch = new SensorSwitchPanel("Encoders",SensorState.DISCONECTED);
	     IMU_switch = new SensorSwitchPanel("IMU",SensorState.DISCONECTED);
	     controlInputs_switch = new SensorSwitchPanel("Control Inputs",SensorState.DISCONECTED);
	     logging_switch = new SensorSwitchPanel("Logging",config.logging);
	     //autonomous_switch = new SensorSwitchPanel("Autonomous",Robot.getInstance().get_autonomus());
	    		 
	    
	    bottom_panel.add(gps_switch.getGraphics());
	    bottom_panel.add(frontCam_switch.getGraphics());
	    bottom_panel.add(backCam_switch.getGraphics());
	    bottom_panel.add(encoders_switch.getGraphics());
	    bottom_panel.add(IMU_switch.getGraphics());
	    bottom_panel.add(controlInputs_switch.getGraphics());
	    bottom_panel.add(logging_switch.getGraphics());
	    //bottom_panel.add(autonomous_switch.getGraphics());

	    this.add(bottom_panel);
	    
	    //update timer should be after everything in the gui has been setup
	    updateTimer = new Timer(10000, new updateTimedHandler());
	    updateTimer.setDelay(1000); //updates every .1 seconds 
	    updateTimer.setRepeats(true);
	    updateTimer.start();
	    
	    
		
	}
	
	//updates the display based on external events
	public void updatePanel(){
		//autonomous_switch.setState(Robot.getInstance().get_running());
		//TODO
		gps_switch.setState(Robot.getInstance().getGpsState());
		gps_switch.updateSensorMessage_lbl(Robot.getInstance().getGpsMsg());

		gps_switch.repaint();
		IMU_switch.setState(Robot.getInstance().getImuState());
	    IMU_switch.updateSensorMessage_lbl(Robot.getInstance().getImuMsg());

		IMU_switch.repaint();
	    frontCam_switch.setState(Robot.getInstance().getFrontCamState());
	    frontCam_switch.updateSensorMessage_lbl("In C++ check window");

	    frontCam_switch.repaint();
	    backCam_switch.setState(Robot.getInstance().getBackCamState());
	    backCam_switch.updateSensorMessage_lbl("In C++ check window");


	    backCam_switch.repaint();
	    encoders_switch.setState(Robot.getInstance().getEncoderState());
	    encoders_switch.updateSensorMessage_lbl(Robot.getInstance().getEncoderMsg());
	    encoders_switch.repaint();
	    controlInputs_switch.setState(Robot.getInstance().getControlInputState());
	    controlInputs_switch.updateSensorMessage_lbl(Robot.getInstance().getControlInputMsg());
	    controlInputs_switch.repaint();
	    //TODO add update for logging_switch
	    
		
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
	
	//timed update for the control input 
	private class updateTimedHandler implements ActionListener
	{
		@Override 
		public void actionPerformed(ActionEvent e)
		{
			updatePanel();		
		}
	}
	
	
	static void updateStartPause_btn(){
		if(config.active)
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
		startPause_btn.repaint();		
		
	}
	
	
	private class StartPauseButtonHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			//inverts the state of the system every time the button is pressed
			if(config.active){
				config.active = false;
			}else{
				config.active = true;
			}
			updateStartPause_btn();

		}
	}

}
