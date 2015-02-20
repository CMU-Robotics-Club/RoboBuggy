/**
 * 
 * Tool for plotting points along the RoboBuggy course
 * @author vivaanbahl
 * Last Update: October 10, 2014
 * 
 */


package pointsTracker;

import javax.swing.JFrame;

public class Main {

	private static JFrame f;
	private static final int DISP_WIDTH = 1000;
	private static final int DISP_HEIGHT = 700;
	
	public static void main(String[] args) {
		f = new JFrame("RoboBuggy Map Plotting Tool");
		f.setSize(DISP_WIDTH, DISP_HEIGHT);
		
		Display display = new Display(DISP_WIDTH, DISP_HEIGHT);
		f.add(display);
		f.setResizable(false);
		
		f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		f.setLayout(null);
		f.setVisible(true);
	}
	
}
