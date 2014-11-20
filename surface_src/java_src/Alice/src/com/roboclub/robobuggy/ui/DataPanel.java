package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;

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

	private GpsPanel gpsPanel;
	
	/* Data Fields */
	private JLabel aX, aY, aZ;
	private JLabel rX, rY, rZ;
	private JLabel mX, mY, mZ;
	private JLabel encTicks;
	private JLabel steeringAng;
	private JLabel errorNum;
	private static final Font dataFont = new Font("sanserif", Font.BOLD, 15);
	
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
		panel.setLayout(new GridLayout(4,6));
		
		aX = new JLabel();
		JLabel label = new JLabel("   aX: ");
		panel.add(label);
		panel.add(aX);
		
		aY = new JLabel();
		label = new JLabel("   aY: ");
		panel.add(label);
		panel.add(aY);
		
		aZ = new JLabel();
		label = new JLabel("   aZ: ");
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
		
		encTicks = new JLabel();
		label = new JLabel("   Ticks: ");
		panel.add(label);
		panel.add(encTicks);
		
		steeringAng = new JLabel();
		label = new JLabel("   Angle: ");
		panel.add(label);
		panel.add(steeringAng);
		
		errorNum = new JLabel();
		label = new JLabel("   Errors: ");
		panel.add(label);
		panel.add(errorNum);
		
		return panel;
	}
	
	// Update Sensor Data in Graph
	public void UpdateAccel(Float aX_, Float aY_, Float aZ_) {
		if (aX_ != null & aX != null) aX.setText(aX.toString());
		if (aY_ != null & aY != null) aY.setText(aY.toString());
		if (aZ_ != null & aZ != null) aZ.setText(aZ.toString());
	}
		
	public void UpdateGyro(Float rX_, Float rY_, Float rZ_) {
		if (rX_ != null & rX != null) rX.setText(rX_.toString());
		if (rZ_ != null & rY != null) rY.setText(rY_.toString());
		if (rY_ != null & rZ != null) rZ.setText(rZ_.toString());
	}
	
	public void UpdateMagnet(Float mX_, Float mY_, Float mZ_) {
		if (mX_ != null & mX != null) mX.setText(mX_.toString());
		if (mY_ != null & mY != null) mY.setText(mY_.toString());
		if (mZ_ != null & mZ != null) mZ.setText(mZ_.toString());
	}
}
