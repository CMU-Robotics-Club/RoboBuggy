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
import com.roboclub.robobuggy.messages.StateMessage;
import com.roboclub.robobuggy.nodes.baseNodes.NodeState;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Publisher;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

/**
 * {@link JPanel} used to represent a switch for a robobuggy sensor
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
	private JButton sensorBtn;
	private Publisher publisher;

	/**
	 * Constructs a new {@link SensorSwitch} object
	 * @param name Name of the panel
	 * @param sensor {@link NodeChannel} of the sensor
	 */
	public SensorSwitch(String name, NodeChannel sensor) {
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridLayout(1,2));

		JLabel sensorName_lbl = new JLabel(name, SwingConstants.CENTER);
		//sensorName_lbl.setFont(new Font("serif", Font.BOLD, 20));
		this.add(sensorName_lbl);

		sensorBtn = new JButton("OFF");
		sensorBtn.setHorizontalTextPosition(SwingConstants.CENTER);
		sensorBtn.setFont(new Font("serif", Font.BOLD, 20));
		sensorBtn.setForeground(Color.WHITE);
		this.add(sensorBtn);
		sensorBtn.addActionListener(new ResetHandler());
		
		publisher = new Publisher(sensor.getRstPath());
		// Subscriber for sensor state changes
		new Subscriber(sensor.getStatePath(), new UpdateListener());
		
		// Default to displaying sensors as not in use
		updateButton(NodeState.NOT_IN_USE);
	}


	private void updateButton(NodeState state) {
		switch (state) {
		case ON:
			sensorBtn.setEnabled(true);
			sensorBtn.setText("ON");
			sensorBtn.setBackground(Color.GREEN);
			break;
		case DISCONNECTED:
			sensorBtn.setEnabled(true);
			sensorBtn.setText("OFF");
			sensorBtn.setBackground(Color.RED);
			break;
		case NOT_IN_USE:
			sensorBtn.setEnabled(false);
			sensorBtn.setText("OFF");
			sensorBtn.setBackground(Color.BLUE);
			break;
		case FAULT:
			sensorBtn.setEnabled(true);
			sensorBtn.setText("FAULT");
			sensorBtn.setBackground(Color.ORANGE);
			break;
		case WATCHDOG_DEAD:
			sensorBtn.setEnabled(true);
			sensorBtn.setText("WATCHDOG");
			sensorBtn.setBackground(Color.PINK);
			break;
		case ERROR:
		default:
			sensorBtn.setEnabled(true);
			sensorBtn.setText("ERROR");
			sensorBtn.setBackground(Color.RED);
		}
	 //   Gui.getInstance().fixPaint();
	}

	/**
	 * Private class used to handle reset messages
	 */
	private class ResetHandler implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			updateButton(NodeState.DISCONNECTED);
			publisher.publish(new ResetMessage());
		}
	}
	
	/**
	 * Private class used to handle update messages
	 */
	private class UpdateListener implements MessageListener {
		@Override
		public void actionPerformed(String topicName, Message m) {
			updateButton(((StateMessage)m).getState());
		}
	}
}
