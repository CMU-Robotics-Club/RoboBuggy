package org.roboclub.robobuggy.testing;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import javax.swing.JFrame;
import javax.swing.JLabel;

import org.roboclub.robobuggy.map.View;

import junit.framework.TestCase;

public class View_Test extends TestCase {

		public void Assert(boolean result){
			if(result == false){
				fail();
			}
		}
		
		public void test1() throws Exception{
			System.out.println("Starting to test visulizeMap");
			View thisView = new View();
			// Add a window listener for close button
			thisView.addWindowListener(new WindowAdapter() {

				public void windowClosing(WindowEvent e) {
					System.exit(0);
				}
			});
			// This is an empty content area in the frame
			JLabel jlbempty = new JLabel("");
			jlbempty.setPreferredSize(new Dimension(175, 100));
			thisView.getContentPane().add(jlbempty, BorderLayout.CENTER);
			thisView.setVisible(true);
			thisView.pack();



			while(thisView.isFocusable()){
				//note this will cause for the programe to hang
			}
			System.out.println("finsihed testing visulizeMap");
		}
}
