package com.roboclub.robobuggy.ui;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JTextArea;

import com.roboclub.robobuggy.main.MessageLevel;
import com.roboclub.robobuggy.messages.ImuMeasurement;
import com.roboclub.robobuggy.messages.RobobuggyLogicExceptionMeasurment;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.SensorChannel;
import com.roboclub.robobuggy.ros.Subscriber;

public class LogicErrorMessageConsole extends RoboBuggyGUIContainer{
	static final MessageLevel[] messageLevels = MessageLevel.values();
	static final int NUMBER_OF_MESSAGES = messageLevels.length;
	boolean[] show_messages;
	JButton[] buttons;
	JTextArea messages;
	
	String messageBuffer[] = new String[10];
	int messageBufferStart = 0;
	int messageBufferEnd = 0;

	JButton EXCEPTION_btn;
	JButton WARNING_btn;
	JButton NOTE_btn;
	public class Header extends RoboBuggyGUIContainer{
		public Header(){
			show_messages = new boolean[NUMBER_OF_MESSAGES];
			buttons = new JButton[NUMBER_OF_MESSAGES];
			//init header buttons
			for(int k = 0;k<NUMBER_OF_MESSAGES;k++){
				show_messages[k] = true;
				buttons[k] = new JButton(messageLevels[k].toString());
			}
			
			
			JLabel label = new JLabel("Logic Errors");
			EXCEPTION_btn = new JButton("SHOW EXCEPTION");
			EXCEPTION_btn.addActionListener(new ActionListener() {
				
				@Override
				public void actionPerformed(ActionEvent e) {
					show_messages[0] = !show_messages[0];
					if(show_messages[0]){
						EXCEPTION_btn.setBackground(Color.green);
						EXCEPTION_btn.setText("SHOW EXCEPTION");
					}else{
						EXCEPTION_btn.setBackground(Color.gray);
						EXCEPTION_btn.setText("DONT SHOW EXCEPTION");

					}
				}
			});
			WARNING_btn = new JButton("SHOW WARNING");		
			WARNING_btn.addActionListener(new ActionListener() {
				
				@Override
				public void actionPerformed(ActionEvent e) {
					show_messages[1] = !show_messages[1];
					if(show_messages[1]){
						WARNING_btn.setBackground(Color.GREEN);
						WARNING_btn.setText("SHOW WARNING");
					}else{
						WARNING_btn.setBackground(Color.GRAY);
						WARNING_btn.setText("DONT SHOW WARNING");
					}
				}
			});
			NOTE_btn = new JButton("SHOW NOTE");
			NOTE_btn.addActionListener(new ActionListener() {
				
				@Override
				public void actionPerformed(ActionEvent e) {
					show_messages[2] = !show_messages[2];
					if(show_messages[2]){
						NOTE_btn.setBackground(Color.GREEN);
						NOTE_btn.setText("SHOW NOTE");
					}else{
						NOTE_btn.setBackground(Color.GRAY);
						NOTE_btn.setText("DONT SHOW NOTE");

					}
					
					
				}
			});
			this.addComponet(label, 0, 0, .25, 1);
			this.addComponet(EXCEPTION_btn, .25, 0, .25, 1);
			this.addComponet(WARNING_btn, .50, 0, .25, 1);
			this.addComponet(NOTE_btn, .75, 0, .25, 1);


		}


	}
	public void addMessage(String newMessage){
		messageBuffer[messageBufferEnd] = newMessage;
		String text = "";
		for(int i = 0;i< messageBuffer.length;i++){
			text+=messageBuffer[(messageBufferStart+i)%messageBuffer.length]+"\n";
		}
		messageBufferEnd = (messageBufferEnd +1)%messageBuffer.length;
		//move start up if need be
		if(messageBufferEnd == messageBufferStart){
			messageBufferStart = (messageBufferStart +1)%messageBuffer.length;
		}
		messages.setText(text);
		Gui.getInstance().fixPaint();
		
	}
	
	
	public LogicErrorMessageConsole(){
		Header header = new Header();
		EXCEPTION_btn.setEnabled(true);
		WARNING_btn.setEnabled(true);
		NOTE_btn.setEnabled(true);
		 messages = new JTextArea();
		this.addComponet(header, 0.0, 0.0, 1.0, .1);
		this.addComponet(messages, 0.0, 0.1, 1.0, .9);
		messages.setText("this is where user defined messages will go\n");
		
		// Subscriber for LogicException updates
		new Subscriber(SensorChannel.LOGIC_EXCEPTION.getMsgPath(), new MessageListener() {
			@Override
			public void actionPerformed(String topicName, Message m) {
				RobobuggyLogicExceptionMeasurment msg = (RobobuggyLogicExceptionMeasurment)m;
				switch(msg.getLevel()){
				case EXCEPTION:
					if(show_messages[0]){
						addMessage(messageLevels[0].toString()+":\t"+msg.getMessage()+"\n");
					}
					break;
				case WARNING:
					if(show_messages[1]){
						addMessage(messageLevels[1].toString()+"\t"+msg.getMessage()+"\n");
					}
					break;
				case NOTE:
					if(show_messages[2]){
						addMessage(messageLevels[2].toString()+"\t"+msg.getMessage()+"\n");
					}
					break;
				default:
					addMessage("something is wrong with  the logic errot console \n");
				}

			    Gui.getInstance().fixPaint();
				
					}
			
				});

		
	}

}
