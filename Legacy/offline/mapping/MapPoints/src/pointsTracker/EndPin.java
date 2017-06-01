package pointsTracker;

import java.awt.Color;

public class EndPin extends Pin {

	public EndPin(int x, int y, double latx, double laty) {
		super(x, y, latx, laty);
		this.pinColor = Color.RED;
	}

}
