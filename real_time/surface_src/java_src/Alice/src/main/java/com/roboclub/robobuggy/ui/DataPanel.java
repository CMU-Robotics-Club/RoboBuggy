package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.messages.BrakeControlMessage;
import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.messages.GpsMeasurement;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.SteeringMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.NodeChannel;
import com.roboclub.robobuggy.ros.Subscriber;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;
import java.awt.Color;
import java.awt.GridLayout;
//import com.roboclub.robobuggy.ui.GpsPanel.LocTuple;

/**
 * {@link RobobuggyGUIContainer} used for displaying sensor data
 * @author Trevor Decker
 * @author Kevin Brennan 
 *
 * @version 0.5
 * 
 * CHANGELOG: NONE
 * 
 * DESCRIPTION: TODO
 */
public class DataPanel extends RobobuggyGUIContainer {

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
	private JLabel commandAng;
	private JLabel bracking;
	private JLabel errorNum;
	private JLabel latitude,longitude;

	/**
	 * Construct a new {@link DataPanel}
	 */
	public DataPanel() {
		gpsPanel = new GpsPanel();
		this.addComponent(gpsPanel, 0, 0, 1, .8);
		this.addComponent(createDataPanel(), 0, .8, 1, .2);
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
		new Subscriber("uiDataPan", NodeChannel.IMU.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				
				ImuMeasurement msg = (ImuMeasurement)m;
				
				// Limit measurement values to 10 characters				
				String tmp = Double.toString(msg.getYaw());
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				aX.setText(tmp);
				
				tmp = Double.toString(msg.getPitch());
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				aY.setText(tmp);

				tmp = Double.toString(msg.getRoll());
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				aZ.setText(tmp);
				
			}
		});
		
		new Subscriber("uiDataPan", NodeChannel.GPS.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				double lat = ((GpsMeasurement)m).getLatitude();
				double longit = ((GpsMeasurement)m).getLongitude();
			    latitude.setText(Double.toString(lat));
			    longitude.setText(Double.toString(longit));
				//latitude and longitude are both needed 
			    Gui.getInstance().fixPaint();
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
		new Subscriber("uiDataPan", NodeChannel.ENCODER.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				EncoderMeasurement msg = (EncoderMeasurement)m;
				
				String tmp = Double.toString(msg.getVelocity());
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				velocity.setText(tmp);
				
				tmp = Double.toString(msg.getDistance());
				if (tmp.length() > MAX_LENGTH) tmp = tmp.substring(0, MAX_LENGTH);
				distance.setText(tmp);
			    Gui.getInstance().fixPaint();
			}
		});
		
		steeringAng = new JLabel();
		label = new JLabel("   Angle: ");
		panel.add(label);
		panel.add(steeringAng);
		
		// Subscriber for drive control updates
		new Subscriber("uiDataPan", NodeChannel.STEERING.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				steeringAng.setText(Double.toString(((SteeringMeasurement)m).getAngle()));
			    Gui.getInstance().fixPaint();
			}
		});
		
		commandAng = new JLabel();
		label = new JLabel("   command: ");
		panel.add(label);
		panel.add(commandAng);
		
		// Subscriber for drive control updates
		new Subscriber("uiDataPan", NodeChannel.STEERING_COMMANDED.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				steeringAng.setText(Double.toString(((SteeringMeasurement)m).getAngle()));
			    Gui.getInstance().fixPaint();
			}
		});
		
		bracking = new JLabel();
		label = new JLabel(" Bracking: ");
		panel.add(label);
		panel.add(bracking);
		new Subscriber("uiDataPan", NodeChannel.BRAKE_CTRL.getMsgPath(), new MessageListener() {
			
			@Override
			public void actionPerformed(String topicName, Message m) {
				bracking.setText(Boolean.toString(((BrakeControlMessage)m).isBrakeEngaged()));
				
			}
		});
		
		
		errorNum = new JLabel();
		label = new JLabel("   Errors: ");
		panel.add(label);
		panel.add(errorNum);
		
		return panel;
	}
	
	//added by Abhinav Girish ; only temporary
	/**
	 * Returns the values of the {@link DataPanel}
	 * @return the values of the {@link DataPanel}
	 */
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

	/**
	 * @return the gps panel
	 */
	public GpsPanel getGpsPanel() {
		return gpsPanel;
	}
	
	
}
