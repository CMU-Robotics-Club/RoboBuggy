package main;

import java.awt.Color;
import java.awt.Font;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.Timer;

public class ControlsPanel extends JPanel {
	private static final long serialVersionUID = -924045896215455343L;
	
	// Big gui objects
	private JButton startPause_btn;
	private JLabel time_lbl;
	private boolean playPauseState;
    private Date startPressedTime;	
    private Timer timer;
    
	public ControlsPanel() {
		//stuff for setting up logging ie start/stop, file name ...
		this.setBorder(BorderFactory.createLineBorder(Color.black));
		this.setLayout(new GridLayout(4, 1));
		startPause_btn = new JButton("Start");
		playPauseState = true;
		startPause_btn.setFont(new Font("serif", Font.PLAIN, 70));
		startPause_btn.setBackground(Color.GREEN);
		StartPauseButtonHandler startPauseHandler = new StartPauseButtonHandler();
		startPause_btn.addActionListener(startPauseHandler);
		JLabel currentFile_lbl = new JLabel("currentFile",SwingConstants.CENTER);
		JLabel newFile_lbl = new JLabel("newFile",SwingConstants.CENTER);
		

		time_lbl = new JLabel("",SwingConstants.CENTER);
		time_lbl.setFont(new Font("sanserif",Font.PLAIN,70));
		
		timer = new Timer(10, new timerHandler());//updates every .01 seconds
		timer.setDelay(100);
	    timer.setRepeats(true);	
	   

		this.add(startPause_btn);
		this.add(currentFile_lbl);
		this.add(newFile_lbl);
		this.add(time_lbl);
	}
	
	DateFormat df = new SimpleDateFormat("HH:mm:ss.S");
	private class timerHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent ae) {
			Date now = new Date();
			long difference = now.getTime() - startPressedTime.getTime();
			time_lbl.setText(df.format(now) + "/" + df.format(new Date(difference)));
			
			repaint();
		}
	}
	private class StartPauseButtonHandler implements ActionListener
	{
		@Override
		public void actionPerformed(ActionEvent e)
		{
			//inverts the state of the system every time the button is pressed 
			playPauseState = !playPauseState;
			if(playPauseState)
			{	
				startPause_btn.setBackground(Color.RED);
				startPause_btn.setText("Pause");
				
				timer.start();
				startPressedTime = new Date();
			} else {
				startPause_btn.setBackground(Color.GREEN);
				startPause_btn.setText("Start");
				timer.stop();
			}
			repaint();
		}
	}

}

