package com.roboclub.robobuggy.main;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Font;
import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextArea;

public class DataPanel extends JPanel {
	private static final long serialVersionUID = 3950373392222628865L;

	/* Data Fields */
	private JTextArea lat;
	private JTextArea lon;
	private JTextArea aX;
	private JTextArea aY;
	private JTextArea aZ;
	private JTextArea rX;
	private JTextArea rY;
	private JTextArea rZ;
	private JTextArea mX;
	private JTextArea mY;
	private JTextArea mZ;
	private JTextArea encAng;
	private JTextArea encDst;
	
	private void addPanel(String label, JTextArea text) {
		JPanel tmp = new JPanel();
		tmp.setLayout(new BorderLayout());
		
		JLabel tmp_lbl = new JLabel(label + ": ");
		tmp_lbl.setFont(new Font("sanserif",Font.BOLD,15));
		tmp.add(tmp_lbl, BorderLayout.WEST);
		
		if (text == null) text = new JTextArea();
		tmp.add(text, BorderLayout.EAST);
		
		this.add(tmp);
	}
	
	public DataPanel(Container parent) {
		this.setLayout(new BoxLayout(this, BoxLayout.Y_AXIS));
		
		lat = new JTextArea();
		this.addPanel("Lat", lat);
		lon = new JTextArea();
		this.addPanel("Lon", lon);
		aX = new JTextArea();
		this.addPanel("aX", aX);
		aY = new JTextArea();
		this.addPanel("aY", aY);
		aZ = new JTextArea();
		this.addPanel("aZ", aZ);
		rX = new JTextArea();
		this.addPanel("rX", rX);
		rY = new JTextArea();
		this.addPanel("rY", rY);
		rZ = new JTextArea();
		this.addPanel("rZ", rZ);
		mX = new JTextArea();
		this.addPanel("mX", mX);
		mY = new JTextArea();
		this.addPanel("mY", mY);
		mZ = new JTextArea();
		this.addPanel("mZ", mZ);
		encAng = new JTextArea();
		this.addPanel("Enc Ang", encAng);
		encDst = new JTextArea();
		this.addPanel("Enc Dst", encDst);
	}
	
	// Update Sensor Data in Graph
	public void UpdatePos(Float lat_, Float lon_) {
		if (lat_ != null & lat != null) {
			lat.setText(lat_.toString());
		}
		if (lon_ != null & lon != null) {
			lon.setText(lon_.toString());
		}
	}
	
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
	
	public void UpdateAngle(Integer ang_) {
		if (ang_ != null & encAng != null) encAng.setText(ang_.toString());
	}
	
	public void UpdateDst(Long dst_) {
		if (dst_ != null & encDst != null) encDst.setText(dst_.toString());
	}
}
