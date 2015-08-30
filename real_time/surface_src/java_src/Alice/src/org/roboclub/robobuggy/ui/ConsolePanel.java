package org.roboclub.robobuggy.ui;

import javax.swing.JPanel;
import javax.swing.JTextArea;

import org.roboclub.robobuggy.ros.Message;
import org.roboclub.robobuggy.ros.MessageListener;
import org.roboclub.robobuggy.ros.SensorChannel;
import org.roboclub.robobuggy.ros.Subscriber;

/**
 * 
 * @author Trevor Decker
 * @author Kevin B
 * @verion 0.0
 *
 */
public class ConsolePanel extends JPanel{
	JTextArea ta = new JTextArea("",30,118); //TODO don't hardcode size 
	Subscriber s = new Subscriber(SensorChannel.GUI_CONSOLE_TEXT.getMsgPath(), new MessageListener() {
		@Override
		public void actionPerformed(String topicName, Message m) {
			//TODO actually print text to the display  
			String current_text = ta.getText();
			//TODO shorten the current text so that it does not become too large 
			String new_text = current_text + "test";
			ta.setText(new_text);
			ta.update(getGraphics());  //don't know if this works or is important 
		}
	});
	
	
	public ConsolePanel(){
		String textToDisplay = "Hello World!";
	    ta.setLineWrap(true);
		this.add(ta);
	}
	
	public void MessageListener(){
		
	}
} 
