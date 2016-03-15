package com.roboclub.robobuggy.ui;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;
import java.net.URISyntaxException;
import java.util.List;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JLabel;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMainFile;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.Util;

/**
 * This is a controlling class for setting the system settings using an easy to user user interface 
 * @author Trevor Decker
 *
 */
public class ConfigurationPanel extends RobobuggyGUIContainer{

	/**
	 * Constructor for the configuration Panel, this is were all of the gui elements 
	 * for the configuration panel are created 
	 */
	public ConfigurationPanel() {
		
		
		addComponent(new JLabel("Config File"), 0, 0, .1, .1);
		JLabel currentConfigFileLabel = new JLabel(RobobuggyConfigFile.CONFIG_FILE);
		addComponent(currentConfigFileLabel ,.3,0,.4,.1);

		JButton ConfigFileSelectBtn = new JButton("Select File");
		
		ConfigFileSelectBtn.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//Create a file chooser
				final JFileChooser fc = new JFileChooser();
				int returnVal = fc.showOpenDialog(null);
				
				//only update the file if a file was selected properly 
				if(returnVal == fc.APPROVE_OPTION){
					File SelectedFile = fc.getSelectedFile();
					RobobuggyConfigFile.CONFIG_FILE = SelectedFile.getAbsolutePath();
					currentConfigFileLabel.setText(RobobuggyConfigFile.CONFIG_FILE);
						RobobuggyMainFile.resetSystem();
					//TODO update stuff in the back end
				}else{
					new RobobuggyLogicNotification("did not select a file properly", RobobuggyMessageLevel.WARNING);
				}
				
			}
		});
		
		addComponent(ConfigFileSelectBtn, .1, 0, .2, .1);
		JButton SaveConfigButton = new JButton("Save Config");
		SaveConfigButton.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				// TODO Auto-generated method stub
			}
		});
		addComponent(SaveConfigButton, .9, 0, .1, .1);
		JButton loadConfigButton = new JButton("Load Config");
		loadConfigButton.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				// TODO Auto-generated method stub
			}
		});
		addComponent(loadConfigButton, .8, 0, .1, .1);
		
		
		addComponent(new JLabel("Way Point File"),0,.1,.1,.1);
		JButton WayPointFileSelectBtn = new JButton("Select File"); 
		addComponent(WayPointFileSelectBtn, .1, .1, .2, .1);
		JLabel currentWayPointLable = new  JLabel(RobobuggyConfigFile.WAYPOINT_SOURCE_LOG_FILE);
		addComponent(currentWayPointLable,.3,.1,.4,.1);

		WayPointFileSelectBtn.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//Create a file chooser
				final JFileChooser fc = new JFileChooser();
				int returnVal = fc.showOpenDialog(null);
				
				//only update the file if a file was selected properly 
				if(returnVal == fc.APPROVE_OPTION){
					File SelectedFile = fc.getSelectedFile();
					RobobuggyConfigFile.WAYPOINT_SOURCE_LOG_FILE = SelectedFile.getAbsolutePath();
					currentWayPointLable.setText(RobobuggyConfigFile.WAYPOINT_SOURCE_LOG_FILE);
					//TODO update stuff in the back end
				}else{
					new RobobuggyLogicNotification("did not select a file properly", RobobuggyMessageLevel.WARNING);
				}				
			}
		});



		addComponent(new JLabel("Play Back Log"),0,.2,.1,.1);
		JButton playBackSlectFileButton = new JButton("Select File");
		addComponent(playBackSlectFileButton, .1, .2, .2, .1);
		JLabel currentPlayBackSourceFileLabel = new JLabel(RobobuggyConfigFile.PLAY_BACK_SOURCE_FILE);
		addComponent(currentPlayBackSourceFileLabel,.3,.2,.4,.1);
		playBackSlectFileButton.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				
				//Create a file chooser
				final JFileChooser fc = new JFileChooser();
				int returnVal = fc.showOpenDialog(null);
				
				//only update the file if a file was selected properly 
				if(returnVal == fc.APPROVE_OPTION){
					File SelectedFile = fc.getSelectedFile();
					RobobuggyConfigFile.PLAY_BACK_SOURCE_FILE = SelectedFile.getAbsolutePath();
					currentPlayBackSourceFileLabel.setText(RobobuggyConfigFile.PLAY_BACK_SOURCE_FILE);
					//TODO update stuff in the back end
				}else{
					new RobobuggyLogicNotification("did not select a file properly", RobobuggyMessageLevel.WARNING);
				}					
			}
		});


		String[] portOptions = getPortOptions();
		
		///////////////////////// IMU ////////////////////////////////
		addComponent(new JLabel("IMU Port"), 0, .4, .1, .05);
		JComboBox imuPortSelector = new JComboBox(portOptions);
		addComponent(imuPortSelector, .1, .4, .2, .05);
		
		imuPortSelector.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO bring down the system 
				int index = imuPortSelector.getSelectedIndex(); //TODO lots of port logic 
				RobobuggyConfigFile.COM_PORT_IMU = portOptions[index];
				if(index == 0){
					//then we decided to select no port so we should not use this sensor 
					RobobuggyConfigFile.IMU_ENABLED = false;
				}else{
					RobobuggyConfigFile.IMU_ENABLED = true;

				}
				
				//TODO bring back up the system
				
			}
		});
		
		/////////////////////////// GPS /////////////////////////////////
		addComponent(new JLabel("GPS Port"), 0, .45, .1, .05);
		JComboBox gpsPortSelector = new JComboBox(portOptions);
		addComponent(gpsPortSelector, .1, .45, .2, .05);
		gpsPortSelector.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO bring down the system 
				int index = gpsPortSelector.getSelectedIndex(); //TODO lots of port logic 
				RobobuggyConfigFile.COM_PORT_GPS = portOptions[index];
				if(index == 0){
					//then we decided to select no port so we should not use this sensor 
					RobobuggyConfigFile.GPS_ENABLED = false;
				}else{
					RobobuggyConfigFile.GPS_ENABLED = true;
				}
				
				//TODO bring back up the system				
			}
		});
		
		///////////////////////////// RBSM ////////////////////////////////
		addComponent(new JLabel("RBSM Port"), 0, .5, .1, .05);
		JComboBox rbsmPortSelector = new JComboBox(portOptions);
		addComponent(rbsmPortSelector, .1, .5, .2, .05);
		rbsmPortSelector.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO bring down the system 
				int index = rbsmPortSelector.getSelectedIndex(); //TODO lots of port logic 
				RobobuggyConfigFile.COM_PORT_RBSM = portOptions[index];
				if(index == 0){
					//then we decided to select no port so we should not use this sensor 
					RobobuggyConfigFile.ENCODER_ENABLED = false;
				}else{
					RobobuggyConfigFile.ENCODER_ENABLED = true;
				}
				
				//TODO bring back up the system						
			}
		});
		
		
		///////////////////////////// VISION System ////////////////////////
		addComponent(new JLabel("Vision System Port"), 0, .55, .1, .05);
		JComboBox visionSystemPortSelector = new JComboBox(portOptions);
		addComponent(visionSystemPortSelector, .1, .55, .2, .05);  //TODO do this for cams
		visionSystemPortSelector.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO bring down the system 
				int index = visionSystemPortSelector.getSelectedIndex(); //TODO lots of port logic 
				RobobuggyConfigFile.PORT_VISION = portOptions[index];
				if(index == 0){
					//then we decided to select no port so we should not use this sensor 
					RobobuggyConfigFile.VISION_SYSTEM_ENABLED = false;
				}else{
					RobobuggyConfigFile.VISION_SYSTEM_ENABLED = true;
				}
				
				//TODO bring back up the system				
			}
		});
		
		
	}
	
	//TODO select port default 
	
	public String[] getPortOptions(){
		List<String> ports = Util.getAvailablePorts();
		String[] portOptions = new String[1+ports.size()];
		portOptions[0] = "NO PORT SELECTED";
		for(int i = 0;i<ports.size();i++){
			portOptions[i+1] = ports.get(i);
		}
		return portOptions;
		
	}
	
	
}
