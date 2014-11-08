package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

import com.roboclub.robobuggy.messages.ResetMessage;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.sensors.SensorState;

/**
 * 
 * @author Trevor Decker
 * @author Kevin Brennan
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class SensorSwitch extends JPanel {
	private static final long serialVersionUID = 8232116275431651229L;
	private JButton sensor_btn;
	private Publisher publisher;

	public SensorSwitch(String sensorName, String publisherName) {
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridLayout(1,2));

		JLabel sensorName_lbl = new JLabel(sensorName, SwingConstants.CENTER);
		sensorName_lbl.setFont(new Font("serif", Font.BOLD, 20));
		this.add(sensorName_lbl);

		sensor_btn = new JButton("OFF");
		sensor_btn.setHorizontalTextPosition(SwingConstants.CENTER);
		sensor_btn.setFont(new Font("serif", Font.BOLD, 20));
		sensor_btn.setForeground(Color.WHITE);
		this.add(sensor_btn);
		sensor_btn.addActionListener(new OnButtonHandler());
		
		publisher = new Publisher(publisherName);
		
		updateButton(SensorState.NOT_IN_USE);
	}


	private void updateButton(SensorState state) {
		switch (state) {
		case ON:
			sensor_btn.setEnabled(true);
			sensor_btn.setText("ON");
			sensor_btn.setBackground(Color.GREEN);
			break;
		case DISCONNECTED:
			sensor_btn.setEnabled(true);
			sensor_btn.setText("OFF");
			sensor_btn.setBackground(Color.RED);
			break;
		case NOT_IN_USE:
			sensor_btn.setEnabled(false);
			sensor_btn.setText("OFF");
			sensor_btn.setBackground(Color.BLUE);
			break;
		case FAULT:
			sensor_btn.setEnabled(true);
			sensor_btn.setText("FAULT");
			sensor_btn.setBackground(Color.ORANGE);
		case ERROR:
		default:
			sensor_btn.setEnabled(true);
			sensor_btn.setText("ERROR");
			sensor_btn.setBackground(Color.RED);
		}
	}

	private class OnButtonHandler implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			updateButton(SensorState.DISCONNECTED);
			publisher.publish(new ResetMessage());
		}
	}
}
