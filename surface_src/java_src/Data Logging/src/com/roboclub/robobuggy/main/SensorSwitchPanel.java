package com.roboclub.robobuggy.main;

import java.awt.Color;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;



public class SensorSwitchPanel {
private JPanel sensor_switch;
private JButton sensor_on_btn;
private JButton sensor_reset_btn;
private JButton sensor_off_btn;


private Sensor_state_type sensor_state;

	SensorSwitchPanel(String sensorName,Sensor_state_type defaultState){
		
		sensor_state = defaultState;
		
		sensor_switch = new JPanel();
		sensor_switch.setBorder(BorderFactory.createLineBorder(Color.black));
		sensor_switch.setLayout(new GridLayout(5, 1));

		JLabel sensorName_lbl = new JLabel(sensorName);
		sensorName_lbl.setHorizontalAlignment(JLabel.CENTER);
		sensor_switch.add(sensorName_lbl);
		
		//TODO setup  label changing by each sensor 
		JLabel sensorMessage_lbl = new JLabel("No Data Avilable");
		sensorMessage_lbl.setHorizontalAlignment(JLabel.CENTER);
		sensor_switch.add(sensorMessage_lbl);
		
		sensor_on_btn = new JButton("ON");
		sensor_switch.add(sensor_on_btn);
		OnButtonHandler  onHandler = new OnButtonHandler();
		sensor_on_btn.addActionListener(onHandler);
		
		sensor_reset_btn = new JButton("RESET");
		sensor_switch.add(sensor_reset_btn);
		ResetButtonHandler  resetHandler = new ResetButtonHandler();
		sensor_reset_btn.addActionListener(resetHandler);
		
		sensor_off_btn = new JButton("OFF");
		sensor_switch.add(sensor_off_btn);
		OffButtonHandler  offHandler = new OffButtonHandler();
		sensor_off_btn.addActionListener(offHandler);
			
		updateButtonColors();


	}

public Sensor_state_type getState(){
	return sensor_state;
}

public void setState(Sensor_state_type newSensor_state){
	sensor_state = newSensor_state;
}
	
private void updateButtonColors(){
	if(sensor_state == Sensor_state_type.ON){
		sensor_on_btn.setBackground(Color.GREEN);
	}else{
		sensor_on_btn.setBackground(Color.LIGHT_GRAY);
	}
	if(sensor_state == Sensor_state_type.RESET){
		sensor_reset_btn.setBackground(Color.ORANGE);
	}else{
		sensor_reset_btn.setBackground(Color.LIGHT_GRAY);
	}
	if(sensor_state == Sensor_state_type.OFF){
		sensor_off_btn.setBackground(Color.RED);
	}else{
		sensor_off_btn.setBackground(Color.LIGHT_GRAY);
	}
}
	
public JPanel getGraphics(){
		return sensor_switch;
	}

private class OnButtonHandler implements ActionListener
{
	@Override
	public void actionPerformed(ActionEvent e)
	{
		//TODO
		sensor_state = Sensor_state_type.ON;
		updateButtonColors();
	}
}

private class ResetButtonHandler implements ActionListener
{
	@Override
	public void actionPerformed(ActionEvent e)
	{
		//TODO
		sensor_state = Sensor_state_type.RESET;
		updateButtonColors();
	}
}

private class OffButtonHandler implements ActionListener
{
	@Override
	public void actionPerformed(ActionEvent e)
	{
		//TODO
		sensor_state = Sensor_state_type.OFF;
		updateButtonColors();
	}
}




	}
	