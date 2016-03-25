package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.RobobuggyConfigFile;
import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMainFile;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.Util;

import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JLabel;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.util.List;

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
		JLabel currentConfigFileLabel = new JLabel(RobobuggyConfigFile.getConfigFile());
		addComponent(currentConfigFileLabel ,.3,0,.4,.1);

		JButton configFileSelectBtn = new JButton("Select File");
		
		configFileSelectBtn.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//Create a file chooser
				final JFileChooser fc = new JFileChooser();
				fc.setCurrentDirectory(new File(RobobuggyConfigFile.getConfigFile()));
				int returnVal = fc.showOpenDialog(null);
				
				//only update the file if a file was selected properly 
				if(returnVal == fc.APPROVE_OPTION){
					File selectedFile = fc.getSelectedFile();
					RobobuggyConfigFile.setConfigFile(selectedFile.getAbsolutePath());
					currentConfigFileLabel.setText(RobobuggyConfigFile.getConfigFile());
						RobobuggyMainFile.resetSystem();
				}else{
					new RobobuggyLogicNotification("did not select a file properly", RobobuggyMessageLevel.WARNING);
				}
				
			}
		});
		
		addComponent(configFileSelectBtn, .1, 0, .2, .1);
		JButton saveConfigButton = new JButton("Save Config");
		saveConfigButton.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
		    	RobobuggyConfigFile.saveConfigFile();

			}
		});
		addComponent(saveConfigButton, .9, 0, .1, .1);
		JButton loadConfigButton = new JButton("Load Config");
		loadConfigButton.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
		    	RobobuggyConfigFile.loadConfigFile();
			}
		});
		addComponent(loadConfigButton, .8, 0, .1, .1);
		
		
		addComponent(new JLabel("Way Point File"),0,.1,.1,.1);
		JButton wayPointFileSelectBtn = new JButton("Select File"); 
		addComponent(wayPointFileSelectBtn, .1, .1, .2, .1);
		JLabel currentWayPointLable = new  JLabel(RobobuggyConfigFile.getWaypointSourceLogFile());
		addComponent(currentWayPointLable,.3,.1,.4,.1);

		wayPointFileSelectBtn.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//Create a file chooser
				final JFileChooser fc = new JFileChooser();
				fc.setCurrentDirectory(new File(RobobuggyConfigFile.getConfigFile()));
				int returnVal = fc.showOpenDialog(null);
				
				//only update the file if a file was selected properly 
				if(returnVal == fc.APPROVE_OPTION){
					File selectedFile = fc.getSelectedFile();
					RobobuggyConfigFile.setWayPointSourceLogFile(selectedFile.getPath());
					currentWayPointLable.setText(RobobuggyConfigFile.getWaypointSourceLogFile());
					//TODO update stuff in the back end
				}else{
					new RobobuggyLogicNotification("did not select a file properly", RobobuggyMessageLevel.WARNING);
				}				
			}
		});



		addComponent(new JLabel("Play Back Log"),0,.2,.1,.1);
		JButton playBackSlectFileButton = new JButton("Select File");
		addComponent(playBackSlectFileButton, .1, .2, .2, .1);
		JLabel currentPlayBackSourceFileLabel = new JLabel(RobobuggyConfigFile.getPlayBackSourceFile());
		addComponent(currentPlayBackSourceFileLabel,.3,.2,.4,.1);
		playBackSlectFileButton.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				
				//Create a file chooser
				final JFileChooser fc = new JFileChooser();
				fc.setCurrentDirectory(new File(RobobuggyConfigFile.getConfigFile()));
				int returnVal = fc.showOpenDialog(null);
				
				//only update the file if a file was selected properly 
				if(returnVal == fc.APPROVE_OPTION){
					File selectedFile = fc.getSelectedFile();
					RobobuggyConfigFile.setPlayBackSourceFile(selectedFile.getAbsolutePath());
					currentPlayBackSourceFileLabel.setText(RobobuggyConfigFile.getPlayBackSourceFile());
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
		imuPortSelector.setSelectedIndex(findPortIndex(RobobuggyConfigFile.getComPortImu(),portOptions));
		addComponent(imuPortSelector, .1, .4, .2, .05);
		
		imuPortSelector.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO bring down the system 
				int index = imuPortSelector.getSelectedIndex(); //TODO lots of port logic 
				RobobuggyConfigFile.setComPortImu(portOptions[index]);
				if(index == 0){
					//then we decided to select no port so we should not use this sensor 
					RobobuggyConfigFile.setImuEnabled(false);
				}else{
					RobobuggyConfigFile.setImuEnabled(true);

				}
				//TODO bring back up the system
				
			}
		});
		
		/////////////////////////// GPS /////////////////////////////////
		addComponent(new JLabel("GPS Port"), 0, .45, .1, .05);
		JComboBox gpsPortSelector = new JComboBox(portOptions);
		gpsPortSelector.setSelectedIndex(findPortIndex(RobobuggyConfigFile.getComPortGPS(),portOptions));
		addComponent(gpsPortSelector, .1, .45, .2, .05);
		gpsPortSelector.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO bring down the system 
				int index = gpsPortSelector.getSelectedIndex(); //TODO lots of port logic 
				RobobuggyConfigFile.setComPortGps(portOptions[index]);
				if(index == 0){
					//then we decided to select no port so we should not use this sensor 
					RobobuggyConfigFile.setGpsEnabled(false);
				}else{
					RobobuggyConfigFile.setGpsEnabled(true);
				}
				
				//TODO bring back up the system				
			}
		});
		
		///////////////////////////// RBSM ////////////////////////////////
		addComponent(new JLabel("RBSM Port"), 0, .5, .1, .05);
		JComboBox rbsmPortSelector = new JComboBox(portOptions);
		rbsmPortSelector.setSelectedIndex(findPortIndex(RobobuggyConfigFile.getComPortRBSM(),portOptions));
		addComponent(rbsmPortSelector, .1, .5, .2, .05);
		rbsmPortSelector.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO bring down the system 
				int index = rbsmPortSelector.getSelectedIndex(); //TODO lots of port logic 
				RobobuggyConfigFile.setComPortRBSM(portOptions[index]);
				if(index == 0){
					//then we decided to select no port so we should not use this sensor 
					RobobuggyConfigFile.setEncoderEnabled(false);
				}else{
					RobobuggyConfigFile.setEncoderEnabled(true);
				}
				
				//TODO bring back up the system						
			}
		});
		
		
		///////////////////////////// VISION System ////////////////////////
		//TODO have the vison system ports reported and make the vision system follow this system
		addComponent(new JLabel("Vision System Port"), 0, .55, .1, .05);
		JComboBox visionSystemPortSelector = new JComboBox(portOptions);
		visionSystemPortSelector.setSelectedIndex(findPortIndex(RobobuggyConfigFile.getPortVision(),portOptions));
		addComponent(visionSystemPortSelector, .1, .55, .2, .05);  //TODO do this for cams
		visionSystemPortSelector.addActionListener(new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				//TODO bring down the system 
				int index = visionSystemPortSelector.getSelectedIndex(); //TODO lots of port logic 
				RobobuggyConfigFile.setPortVision(portOptions[index]);
				if(index == 0){
					//then we decided to select no port so we should not use this sensor 
					RobobuggyConfigFile.setVisionSystemEnabled(false);
				}else{
					RobobuggyConfigFile.setVisionSystemEnabled(true);
				}
				
				//TODO bring back up the system				
			}
		});
		
		
	}
	
	
	/**
	 * evaluates to index of when the string port is in the portList if the string port is in the portList
	 * otherwise evaluate to 0
	 * @param port the string of the port you are trying to match
	 * @param portList the string array of all of the strings of ports that you are trying to find port in 
	 * @return the index of port in portList or 0
	 */
	public int findPortIndex(String port,String[] portList){
		int result = 0; //the default should be 0, since it represents no port selected
		for(int i = 1;i<portList.length;i++){
			if(portList[i].equals(port)){
				result = i;
			}
		}
		return result;
	}
	
	/**
	 * Evaluates to a string array of all available com ports and the option "NO PORT SELECTED"
	 * @return evaluates to the string of port options 
	 */
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
