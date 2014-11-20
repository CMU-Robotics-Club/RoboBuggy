package pointsTracker;

import java.awt.Component;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.ArrayList;

import javax.swing.ButtonGroup;
import javax.swing.JComponent;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTextPane;

public class Display extends JComponent implements ActionListener, KeyListener {
	
	private GraphPanel mapCover;
	private JScrollPane pinList;
	private int bottomOfContent = 0;
	private ArrayList<JComponent> pins;
	// Note that a final field can be initialized in constructor
	private final int DISPLAY_WIDTH;   
	private final int DISPLAY_HEIGHT;


	public Display(int width, int height) {
		DISPLAY_WIDTH = width;
		DISPLAY_HEIGHT = height;
		init();
	}


	public void init() {
		mapCover = new GraphPanel(DISPLAY_WIDTH - 200, DISPLAY_HEIGHT);
		mapCover.setBounds(0, 60, mapCover.WIDE, mapCover.HIGH);
		setSize(DISPLAY_WIDTH, DISPLAY_HEIGHT);
		initRadioGroup();
		add(mapCover);
		
		pins = new ArrayList<JComponent>();
		
		pinList = new JScrollPane();
		pinList.setBounds(DISPLAY_WIDTH - 200, 60, 200, DISPLAY_HEIGHT - 60);
		pinList.setEnabled(true);
		pinList.setPreferredSize(new Dimension(pinList.getWidth(), DISPLAY_HEIGHT * 4));
		add(pinList);
		
		
		addKeyListener(this);
		this.setFocusable(true);
		this.requestFocus();
		repaint();
	}
	
	public void addPinToList(Pin p) {
		System.out.println("adding pin to list");
		// TODO Auto-generated method stub
		int posToAdd = bottomOfContent;
		JTextPane toAdd = new JTextPane();
		toAdd.setBounds(0, 0, pinList.getWidth(), 100);
		toAdd.setText("Starter text. If you see this, there's a problem.");
		
		if(p instanceof StartPin){
			posToAdd = 0;
			toAdd.setText("Start:\n    Point A, Point B\n    LatPoint A, LatPoint B");
		}
		else if(p instanceof EndPin) {
			toAdd.setText("End:\n    Point A, Point B\n    LatPoint A, LatPoint B");
		}
		else {
			toAdd.setText("Marker i:\n    Point A, Point B\n    LatPoint A, LatPoint B");
		}
		
		pins.add(toAdd);
		
		redrawPinList();
	}
	
	public void redrawPinList() {
		pinList.removeAll();
		int compHeight = 100;
		for (int i = 0; i < pins.size(); i++) {
			int posY = i * compHeight;
			pins.get(i).setBounds(0, posY, pinList.getWidth(), compHeight);
			pinList.add(pins.get(i));
		}
		pinList.repaint();
	}

	private void initRadioGroup() {
		// TODO Auto-generated method stub
		JRadioButton startButton = new JRadioButton("Start");
		startButton.setBounds(0, 10, 100, 20);
		startButton.setActionCommand("start");

	    JRadioButton midButton = new JRadioButton("Middle");
	    midButton.setBounds(100, 10, 100, 20);
	    midButton.setActionCommand("mid");

	    JRadioButton endButton = new JRadioButton("End");
	    endButton.setBounds(200, 10, 100, 20);
	    endButton.setActionCommand("end");
	    
	    JRadioButton noneButton = new JRadioButton("None");
	    noneButton.setBounds(300, 10, 100, 20);
	    noneButton.setActionCommand("none");

	    //Group the radio buttons.
	    ButtonGroup group = new ButtonGroup();
	    group.add(startButton);
	    group.add(midButton);
	    group.add(endButton);
	    group.add(noneButton);
	    
	    add(startButton);
	    add(midButton);
	    add(endButton);
	    add(noneButton);
	    

	    //Register a listener for the radio buttons.
	    startButton.addActionListener(this);
	    midButton.addActionListener(this);
	    endButton.addActionListener(this);
	    noneButton.addActionListener(this);
	}


	public void paintComponent(Graphics g) { //VERY inefficient way, look into a better method of requesting focus
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		requestFocus();
		mapCover.repaint();
	}


	@Override
	public void actionPerformed(ActionEvent arg0) {
		// TODO Auto-generated method stub
		System.out.println(arg0.getActionCommand());
		if(arg0.getActionCommand().equals("start")) mapCover.setPinMode(GraphPanel.PIN_MODE_START);
		else if(arg0.getActionCommand().equals("mid")) mapCover.setPinMode(GraphPanel.PIN_MODE_MID);
		else if(arg0.getActionCommand().equals("end")) mapCover.setPinMode(GraphPanel.PIN_MODE_END);
		else mapCover.setPinMode(GraphPanel.PIN_MODE_OOPS);
		
	}
	
	 @Override
		public void keyPressed(KeyEvent arg0) { //Turns out this doesn't actually work yet... :D
	    	System.out.println("key pressed");
	    	if(arg0.getKeyCode() == KeyEvent.VK_BACK_SPACE){
	    		System.out.println("backspaaaaaace");
	    		mapCover.removeSelected();
	    	}
		}

		@Override
		public void keyReleased(KeyEvent arg0) {
			// TODO Auto-generated method stub
			
		}

		@Override
		public void keyTyped(KeyEvent arg0) {
			// TODO Auto-generated method stub
			
		}


		
    


}