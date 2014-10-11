package pointsTracker;

import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.image.BufferedImage;
import java.beans.PropertyChangeListener;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.Action;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JRadioButton;

public class Display extends JComponent implements ActionListener {
	
	private final int DISP_WIDTH;
	private final int DISP_HEIGHT;
	
	private Map map;
	

	public Display(int dispWidth, int dispHeight) {
		// TODO Auto-generated constructor stub
		DISP_WIDTH = dispWidth;
		DISP_HEIGHT = dispHeight;
		
		init();
	}
	
	
	
	public void paintComponent(Graphics g) {
		// TODO Auto-generated method stub
//		repaint();
	}



	private void init() {
		setSize(DISP_WIDTH, DISP_HEIGHT);
		BufferedImage img = null;
		
		try {
			img = ImageIO.read(ClassLoader.getSystemResourceAsStream("courseMap.png"));
			//777 x 539
		} catch (IOException e) {
			// TODO Auto-generated catch block
			System.out.println("uhoh");
		}
		
		map = new Map(777, 539, img);
		map.setVisible(true);
		this.add(map);
		map.repaint();
		
		
		initRadioGroup();
		initClearButton();
		
	}



	private void initClearButton() {
		// TODO Auto-generated method stub
		JButton clear = new JButton("Clear Pins");
		clear.setText("Clear Pins");
		clear.setAction(new Action() {
			
			@Override
			public void actionPerformed(ActionEvent arg0) {
				// TODO Auto-generated method stub
				Display.this.map.clearPins();
				Display.this.map.repaint();
			}
			
			@Override
			public void setEnabled(boolean arg0) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void removePropertyChangeListener(PropertyChangeListener arg0) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void putValue(String arg0, Object arg1) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public boolean isEnabled() {
				// TODO Auto-generated method stub
				return false;
			}
			
			@Override
			public Object getValue(String arg0) {
				// TODO Auto-generated method stub
				return null;
			}
			
			@Override
			public void addPropertyChangeListener(PropertyChangeListener arg0) {
				// TODO Auto-generated method stub
				
			}
		});
		clear.setEnabled(true);
		clear.setBounds(map.getX() + map.getWidth() + 20, 80, 150, 40);
		add(clear);
		clear.repaint();
		
	}



	private void initRadioGroup() {
		// TODO Auto-generated method stub
		JRadioButton startpin = new JRadioButton("Start Pin");
		startpin.setActionCommand("start");
		startpin.addActionListener(this);
		startpin.setSelected(true);
		
		JRadioButton midpin = new JRadioButton("Mid Pin");
		midpin.setActionCommand("mid");
		midpin.addActionListener(this);
		midpin.setSelected(false);
		
		JRadioButton endpin = new JRadioButton("End Pin");
		endpin.setActionCommand("end");
		endpin.addActionListener(this);
		endpin.setSelected(false);
		
		ButtonGroup group = new ButtonGroup();
		group.add(startpin);
		group.add(midpin);
		group.add(endpin);
		
		startpin.setBounds(map.getX() + map.getWidth() + 20, 20, 100, 20);
		add(startpin);
		midpin.setBounds(map.getX() + map.getWidth() + 20, startpin.getY() + startpin.getHeight(), 100, 20);
		add(midpin);
		endpin.setBounds(map.getX() + map.getWidth() + 20, midpin.getY() + midpin.getHeight(), 100, 20);
		add(endpin);
		
	}



	@Override
	public void actionPerformed(ActionEvent e) {
		// TODO Auto-generated method stub
		String pinmode = e.getActionCommand();
		if (pinmode.equals("start")) {
			map.changePinMode(Map.START_PIN);
		}
		else if (pinmode.equals("mid")) {
			map.changePinMode(Map.MID_PIN);
		}
		else if (pinmode.equals("end")) {
			map.changePinMode(Map.END_PIN);
		}
		else {
			map.changePinMode(Map.OOPS_PIN);
		}
	}

}
