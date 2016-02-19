package com.roboclub.robobuggy.ui;

import java.io.File;

import javax.swing.*;

import com.roboclub.robobuggy.main.RobobuggyLogicNotification;
import com.roboclub.robobuggy.main.RobobuggyMessageLevel;
import com.roboclub.robobuggy.main.Robot;

public class VelocityPlotter {

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
	
	public static void printArray(double[] A){
		System.out.print("{");
		for(int x = 0; x < A.length; x++){
			System.out.print(A[x]);
			if(x < A.length-1){
				System.out.print(", ");
			}
		}
		System.out.println("}");
	}
	
	public static void rotateYPR(double[] xyz, double yaw, double pitch, double roll){
		//I assume positive yaw is a right turn
		//positive pitch is upward
		//and positive roll is right side down
		//as per Wikipedia's definition

		//I define +x as to the right on screen, +y as down on screen, and +z as out of the screen
		//I assume the buggy is facing right (+x direction)

		//Also, I'm using aliasing intentionally here

		//Can I really just keep applying transformations in series like this?

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