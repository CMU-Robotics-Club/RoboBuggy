package pointsTracker;

import java.awt.Color;

public class Pin extends Object {
	
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
		pinWidth = 10;
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
}
