package com.roboclub.robobuggy.ui;

import com.roboclub.robobuggy.main.Robot;

/**
 * Stand-alone file to test my code
 * @author davidneiman
 *
 */
public class VelocityPlotter {

	/**
	 * Main file to test stuff (seriously, why do I need a JavaDoc comment here?)
	 * @param args - Same argument that always gets passed to the SPVM
	 * @throws InterruptedException - Same exception that all these main files throw
	 */
	public static void main(String[] args) throws InterruptedException {
		Robot.getInstance(); //Uncomment
        //Gui g = Gui.getInstance();
		
		//Code pulled from the Gui constructor (uncomment)
		RobobuggyJFrame mainWindow = new RobobuggyJFrame("MainWindow",1.0,1.0);	
		RobobuggyGUITabs tabs = new RobobuggyGUITabs();
		
		tabs.addTab(new ImuVisualWindow(), "IMU");
		tabs.addTab(new VelocityWindow(), "Velocity"); //Move later, later
		
		tabs.addTab(new MainGuiWindow(),"Home");
		tabs.addTab(new NodeViewer(),"Nodes");

		mainWindow.addComponent(tabs, 0.0, 0.0, 1.0, 1.0);
		while(true){
			mainWindow.repaint();
		}
		
		

		
		/**/
	}
	
	/**
	 * Utility function to print an array
	 * (Mostly for debugging, as is this whole file)
	 * @param myA - the array being printed
	 */
	public static void printArray(double[] myA){
		System.out.print("{");
		for(int x = 0; x < myA.length; x++){
			System.out.print(myA[x]);
			if(x < myA.length-1){
				System.out.print(", ");
			}
		}
		System.out.println("}");
	}
	
	/**
	 * Rotates a point
	 * I define +x to be to the right (on screen), +y down, and +z out of the screen (right-handed coordinates)
	 * I assume the buggy is initially facing right
	 * Yaw/pitch/roll definitions from Wikipedia
	 * 
	 * I'm using aliasing intentionally
	 * 
	 * @param xyz - length 3 array with the x, y, and z coordinates to be rotated
	 * @param yaw - yaw value to rotate by (positive yaw = right turn)
	 * @param pitch - pitch value to rotate by (positive pitch = nose points upward)
	 * @param roll - roll value to rotate by (positive roll = right wing/side rotates down)
	 */
	public static void rotateYPR(double[] xyz, double yaw, double pitch, double roll){

		//Do yaw:
		double temp = xyz[0]*Math.cos(yaw) - xyz[2]*Math.sin(yaw);
		xyz[2] = xyz[2]*Math.cos(yaw) + xyz[0]*Math.sin(yaw);
		xyz[0] = temp;printArray(xyz);
		//Yaw done

		//Do pitch:
		temp = xyz[0]*Math.cos(pitch) + xyz[1]*Math.sin(pitch); //Remember, +y is DOWN!!!
		xyz[1] = xyz[1]*Math.cos(pitch) - xyz[0]*Math.sin(pitch);
		xyz[0] = temp;printArray(xyz);
		//Pitch done (though possibly incorrect)

		//Do roll:
		temp = xyz[1]*Math.cos(roll) + xyz[2]*Math.sin(roll);
		xyz[2] = xyz[2]*Math.cos(roll) - xyz[1]*Math.sin(roll);
		xyz[1] = temp;printArray(xyz);
		//Roll done
	}
}