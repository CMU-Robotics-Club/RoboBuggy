package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;

import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;
//import com.roboclub.robobuggy.ui.GpsPanel.LocTuple;

/**
 * @author Trevor Decker
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */

public class DataPanel extends JPanel {
	private static final long serialVersionUID = 3950373392222628865L;

	private static final int MAX_LENGTH = 10;
	
	private GpsPanel gpsPanel;
	
	/* Data Fields */
	private JLabel aX, aY, aZ;
	private JLabel rX, rY, rZ;
	private JLabel mX, mY, mZ;
	private JLabel velocity;
	private JLabel distance;
	private JLabel steeringAng;
	private JLabel errorNum;
	private JLabel latitude,longitude;
	private Subscriber gpsSub;
	
	public DataPanel() {
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridBagLayout());
		
		GridBagConstraints gbc = new GridBagConstraints();
		
		gbc.fill = GridBagConstraints.BOTH;
		gbc.gridx = 0;
		gbc.gridy = 0;
		gbc.weightx = 0.34;
		gbc.weighty = 1.0;
		gpsPanel = new GpsPanel();
		this.add(gpsPanel, gbc);
		
		gbc.gridx = 1;
		gbc.weightx = 0.66;
		this.add(createDataPanel(), gbc);
	}
	
	private JPanel createDataPanel() {
		JPanel panel = new JPanel();
		panel.setBorder(BorderFactory.createLineBorder(Color.black));
		panel.setLayout(new GridLayout(5,6));
		
		aX = new JLabel();
		JLabel label = new JLabel("   yaw: ");
		panel.add(label);
		panel.add(aX);
		
		aY = new JLabel();
		label = new JLabel("   pitch: ");
		panel.add(label);
		panel.add(aY);
		
		aZ = new JLabel();
		label = new JLabel("   roll: ");
		panel.add(label);
		panel.add(aZ);
		
		rX = new JLabel();
		label = new JLabel("   rX: ");
		panel.add(label);
		panel.add(rX);
		
		rY = new JLabel();
		label = new JLabel("   rY: ");
		panel.add(label);
		panel.add(rY);
		
		rZ = new JLabel();
		label = new JLabel("   rZ: ");
		panel.add(label);
		panel.add(rZ);
		
		mX = new JLabel();
		label = new JLabel("   mX: ");
		panel.add(label);
		panel.add(mX);
		
		mY = new JLabel();
		label = new JLabel("   mY: ");
		panel.add(label);
		panel.add(mY);
		
		mZ = new JLabel();
		label = new JLabel("   mZ: ");
		panel.add(label);
		panel.add(mZ);
		
		latitude = new JLabel();
		label = new JLabel("  lat: ");
		panel.add(label);
		panel.add(latitude);
		
		longitude = new JLabel();
		label = new JLabel(" long: ");
		panel.add(label);
		panel.add(longitude);
		
		// Subscriber for Imu updates
		new Subscriber(SensorChannel.IMU.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				ImuMeasurement msg = (ImuMeasurement)m;
				
				// Limit measurement values to 10 characters
				String tmp = new String();
				
				tmp = Double.toString(msg.yaw);
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				aX.setText(tmp);
				
				tmp = Double.toString(msg.pitch);
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				aY.setText(tmp);

				tmp = Double.toString(msg.roll);
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				aZ.setText(tmp);
				
			}
		});
		
		gpsSub = new Subscriber(SensorChannel.GPS.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				double lat = ((GpsMeasurement)m).latitude;
				double longit = ((GpsMeasurement)m).longitude;
			    latitude.setText(Double.toString(lat));
			    longitude.setText(Double.toString(longit));
				//latitude and longitude are both needed 
			}
		});		
		
		velocity = new JLabel();
		label = new JLabel("   Velocity: ");
		panel.add(label);
		panel.add(velocity);
		
		distance = new JLabel();
		label = new JLabel("   Distance: ");
		panel.add(label);
		panel.add(distance);
		
		// Subscriber for encoder updates
		new Subscriber(SensorChannel.ENCODER.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				EncoderMeasurement msg = (EncoderMeasurement)m;
				
				String tmp = Double.toString(msg.velocity);
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				velocity.setText(tmp);
				
				tmp = Double.toString(msg.distance);
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				distance.setText(tmp);
			}
		});
		
		steeringAng = new JLabel();
		label = new JLabel("   Angle: ");
		panel.add(label);
		panel.add(steeringAng);
		
		// Subscriber for drive control updates
		new Subscriber(SensorChannel.STEERING.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				steeringAng.setText(Integer.toString(((SteeringMeasurement)m).angle));
			}
		});
		
		//Subscriber for GPS data - agirish
		new Subscriber(SensorChannel.GPS.getMsgPath(), new MessageListener() {
			public void actionPerformed(String topicName, Message m) {
				aX.setText(Double.toString(((GpsMeasurement)m).latitude)); //adding a value 
				
			}
		});
		
		errorNum = new JLabel();
		label = new JLabel("   Errors: ");
		panel.add(label);
		panel.add(errorNum);
		
		return panel;
	}
	
	//added by Abhinav Girish ; only temporary
	public String getValues()
	{
		String values = "";
		values += "aX: "+aX.getText() + " ";
		values += "aY: "+aY.getText() + " ";
		values += "aZ: "+aZ.getText() + " ";
		values += "rX: "+rX.getText() + " ";
		values += "rY: "+rY.getText() + " ";
		values += "rZ: "+rZ.getText() + " ";
		values += "mX: "+mX.getText() + " ";
		values += "mY: "+mY.getText() + " ";
		values += "mZ: "+mZ.getText() + " ";
		
		return values;
	}
	
	
}
