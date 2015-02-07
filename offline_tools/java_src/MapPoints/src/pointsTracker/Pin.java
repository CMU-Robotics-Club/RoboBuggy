package pointsTracker;

import java.awt.Color;
import java.awt.Point;
import java.awt.Rectangle;
import java.awt.Shape;
import java.awt.geom.AffineTransform;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

public class Pin implements Shape{
	
	protected Color pinColor;
	private int screen_x;
	private int screen_y;
	private double lat_x;
	private double lat_y;
	private int pinWidth;
	
	
	public Pin(int x, int y, double latx, double laty) {
		screen_x = x;
		screen_y = y;
		lat_x = latx;
		lat_y = laty;
		
		pinColor = Color.YELLOW;
		pinWidth = 5;
	}
	
	public void setPinLocation(Point loc) {
		screen_x = loc.x;
		screen_y = loc.y;
	}
	
	public void setPinLatLng(double lat, double lon) {
		lat_x = lat;
		lat_y = lon;
	}
	
	public String getCoordinates(){
		return lat_x + ", " + lat_y;
	}
	
	public int getScreenX() {
		return screen_x;
	}

	public int getScreenY() {
		return screen_y;
	}
	
	public Color getPinColor() {
		return pinColor;
	}
	
	public int getPinWidth() {
		return pinWidth;
	}

	@Override
	public boolean contains(Point2D arg0) {
		if(arg0.getX() < screen_x) {
			return false;
		}
		if(arg0.getX() > screen_x + pinWidth) {
			return false;
		}
		if(arg0.getY() < screen_y) {
			return false;
		}
		if(arg0.getY() > screen_y + pinWidth) {
			return false;
		}
		return true;
	}

	@Override
	public boolean contains(Rectangle2D arg0) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean contains(double arg0, double arg1) {
		if(arg0 < screen_x) {
			return false;
		}
		if(arg0 > screen_x + pinWidth) {
			return false;
		}
		if(arg1 < screen_y) {
			return false;
		}
		if(arg1 > screen_y + pinWidth) {
			return false;
		}
		return true;
	}

	@Override
	public boolean contains(double arg0, double arg1, double arg2, double arg3) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public Rectangle getBounds() {
		// TODO Auto-generated method stub
		return new Rectangle(screen_x, screen_y, pinWidth, pinWidth);
	}

	@Override
	public Rectangle2D getBounds2D() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public PathIterator getPathIterator(AffineTransform arg0) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public PathIterator getPathIterator(AffineTransform arg0, double arg1) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public boolean intersects(Rectangle2D arg0) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean intersects(double arg0, double arg1, double arg2, double arg3) {
		// TODO Auto-generated method stub
		return false;
	}

}
