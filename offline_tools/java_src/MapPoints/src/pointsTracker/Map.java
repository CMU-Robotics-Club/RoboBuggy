package pointsTracker;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.util.ArrayList;

import javax.swing.JComponent;

public class Map extends JComponent implements MouseListener, MouseMotionListener {
	
	private final int MAP_WIDTH;
	private final int MAP_HEIGHT;
	
	public static final int START_PIN = 0;
	public static final int MID_PIN = 1;
	public static final int END_PIN = 2;
	public static final int OOPS_PIN = -1;
	
	private int pinMode;
	
	private Image map;
	private ArrayList<Pin> pins;
	
	
	public Map(int width, int height, Image img) {
		MAP_WIDTH = width;
		MAP_HEIGHT = height;
		map = img;
		
		init();
	}
	
	private void init() {
		setSize(MAP_WIDTH, MAP_HEIGHT);
		addMouseListener(this);
		pins = new ArrayList<Pin>();
		pinMode = Map.START_PIN;
		
		repaint();
	}
	
	@Override
	protected void paintComponent(Graphics f) {
		// TODO Auto-generated method stub
		Graphics2D g = (Graphics2D)f;
		
		g.drawImage(map, 0, 0, MAP_WIDTH, MAP_HEIGHT, this);
		
		g.setColor(Color.YELLOW);
		g.setStroke(new BasicStroke(10));
		for (int i = 1; i < pins.size(); i++) {
			Pin p1 = pins.get(i - 1);
			Pin p2 = pins.get(i);
			
			g.drawLine(p1.getScreenX(), p1.getScreenY(), p2.getScreenX(), p2.getScreenY());
		}
		
		g.setStroke(new BasicStroke(3));
		
		for (Pin p : pins) {
			int pinx = p.getScreenX();
			int piny = p.getScreenY();
			int pinw = p.getPinWidth();
			Color pinc = p.getPinColor();
			
			g.setColor(Color.BLACK);
			g.drawOval(pinx - pinw/2, piny - pinw/2, pinw, pinw);
			g.setColor(pinc);
			g.fillOval(pinx - pinw/2, piny - pinw/2, pinw, pinw);
		}
		
	}
	
	public void useMap(Image img){
		map = img;
	}

	public void clearPins() {
		pins = new ArrayList<Pin>();
	}
	
	
	@Override
	public void mouseDragged(MouseEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseMoved(MouseEvent arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseClicked(MouseEvent e) {
		// TODO Auto-generated method stub
		int x = e.getX();
		int y = e.getY();
		int lat = 2*x; //fill these in with actual values later
		int lon = 2*y;
		
		switch (pinMode) {
		case Map.START_PIN:
			makeNewStart(x, y, lat, lon);
			break;
		case Map.MID_PIN:
			makeNewMid(x, y, lat, lon);
			break;
		case Map.END_PIN:
			makeNewEnd(x, y, lat, lon);
			break;
		default:
			System.out.println("OOPS OOPS OOPS");
			break;
		}
		
		repaint();
	}

	private void makeNewEnd(int x, int y, int lat, int lon) {
		// TODO Auto-generated method stub
		int lastInd = pins.size() - 1;
		if (pins.size() == 0) {
			return;
		}
		else if(!(pins.get(lastInd) instanceof EndPin)) {
			pins.add(new EndPin(x, y, lat, lon));
		}
		else {
			pins.remove(lastInd);
			pins.add(new EndPin(x, y, lat, lon));
		}
	}

	private void makeNewMid(int x, int y, int lat, int lon) {
		// TODO Auto-generated method stub
		if (pins.size() == 0) {
			return;
		}
		else if (pins.size() > 2 && (pins.get(pins.size() - 1) instanceof EndPin)) {
			pins.add(pins.size() - 1, new Pin(x, y, lat, lon));
		}
		else if (pins.size() == 1) {
			return;
		}
		else if (pins.size() == 2) {
			pins.add(1, new Pin(x, y, lat, lon));
		}
		
		
	}

	private void makeNewStart(int x, int y, int lat, int lon) {
		// TODO Auto-generated method stub
		if (pins.size() == 0) {
			pins.add(new StartPin(x, y, lat, lon));
		}
		else if(!(pins.get(0) instanceof StartPin)) {
			pins.add(0, new StartPin(x, y, lat, lon));
		}
		else {
			pins.remove(0);
			pins.add(new StartPin(x, y, lat, lon));
		}
	}

	@Override
	public void mouseEntered(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseExited(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mousePressed(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void mouseReleased(MouseEvent e) {
		// TODO Auto-generated method stub
		
	}
	
	/**
	 * changes the pin mode
	 * @param mode accepts any of: 
	 * Map.START_PIN, 
	 * Map.MID_PIN, 
	 * and Map.END_PIN.
	 * If Map.OOPS_PIN is passed, there's a problem. A major problem.
	 */
	public void changePinMode(int mode) {
		pinMode = mode;
	}
	
}
