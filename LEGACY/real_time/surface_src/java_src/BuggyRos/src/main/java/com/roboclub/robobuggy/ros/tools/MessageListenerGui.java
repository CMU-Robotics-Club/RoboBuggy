package com.roboclub.robobuggy.ros.tools;

/**
 * @author Matt Sebek
 *
 * @version 0.5
 *
 * CHANGELOG: NONE
 *
 * DESCRIPTION: Disaply for seeing the contents of messages as they are broadcasted
 */

import java.awt.BorderLayout; //for layout managers and more
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener; //for action events
import java.util.ArrayList;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;
import javax.swing.SwingWorker;
import javax.swing.UIManager;
import javax.swing.table.DefaultTableModel;

//import com.roboclub.robobuggy.messages.EncoderMeasurement;
import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Subscriber;

public class MessageListenerGui extends JPanel implements ActionListener {
	private static final long serialVersionUID = -3528970717084422491L;
	private JTextField filterTextField = new JTextField(256);
	private DefaultTableModel model;

	private class myWorker extends SwingWorker<Object, String> {

		@Override
		protected Object doInBackground() throws Exception {
			System.out.println("do in background fired!");
			Subscriber s = new Subscriber("messageListenerGui", "/sensor/encoder",
					new MessageListener() {
						@Override
						public void actionPerformed(String topicName, Message m) {
							System.out.println("IT RAN I AM SO HAPPY");
						}
					});
			while (true) {
				// TODO fix this.
				Thread.sleep(100);
			}
		}
	}

	public MessageListenerGui(List<String> topic_to_listen_on) {
		setLayout(new BorderLayout());

		// Set up topic-add text box
		JLabel textFieldLabel = new JLabel("Filter:");
		textFieldLabel.setLabelFor(filterTextField);
		filterTextField.addActionListener(this);

		// Lay out the text controls and the labels.
		JPanel controlPane = new JPanel();
		GridBagLayout gridbag = new GridBagLayout();
		GridBagConstraints c = new GridBagConstraints();
		controlPane.setLayout(gridbag);
		c.gridwidth = GridBagConstraints.REMAINDER;
		c.anchor = GridBagConstraints.WEST;
		c.weightx = 1.0;
		controlPane.add(filterTextField, c);
		controlPane.setBorder(BorderFactory.createCompoundBorder(BorderFactory
				.createTitledBorder("Type a prefix then press enter"),
				BorderFactory.createEmptyBorder(5, 5, 5, 5)));

		// Set up table to display messages in the bottom pane
		model = new DefaultTableModel();
		model.addColumn("Time");
		model.addColumn("Message");
		JTable table = new JTable(model);
		JScrollPane tableScroll = new JScrollPane(table);
		tableScroll
				.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
		tableScroll.setBorder(BorderFactory.createCompoundBorder(BorderFactory
				.createCompoundBorder(
						BorderFactory.createTitledBorder("Plain Text"),
						BorderFactory.createEmptyBorder(5, 5, 5, 5)),
				tableScroll.getBorder()));

		// Put everything together.
		JPanel mainPanel = new JPanel(new BorderLayout());
		mainPanel.add(controlPane, BorderLayout.PAGE_START);
		mainPanel.add(tableScroll, BorderLayout.CENTER);
		add(mainPanel, BorderLayout.LINE_START);
	}

	// Listen on filterTextField to have enter hit
	@Override
	public void actionPerformed(ActionEvent e) {
		String input = filterTextField.getText();
		System.out.println("Adding topic..." + input);
		SwingWorker<Object, String> temp = new myWorker();
		temp.execute();
	}

	/**
	 * Create the GUI and show it. For thread safety, this method should be
	 * invoked from the event dispatch thread.
	 */
	private static void createAndShowGUI() {
		// Create and set up the window.
		JFrame frame = new JFrame("Message Monitor");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		// Add content to the window.
		frame.add(new MessageListenerGui(new ArrayList<String>()));

		// Display the window.
		frame.pack();
		frame.setVisible(true);
	}

	public void startGui() {
		System.out.println("code was run!");
		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				// Turn off metal's use of bold fonts
				System.out.println("runlater was run!");
				createAndShowGUI();
			}
		});
	}

	public static void open_message_window(List<String> topics_to_listen_on) {
		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				// Turn off metal's use of bold fonts
				UIManager.put("swing.boldMetal", Boolean.FALSE);
				JFrame frame = new JFrame("ROS Message Monitor");
				frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
				frame.add(new MessageListenerGui(topics_to_listen_on));

				// Display the window.
				frame.pack();
				frame.setVisible(true);
			}
		});

	}

	public static void main(String[] args) {
		// Start the GUI
		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				// Turn off metal's use of bold fonts
				UIManager.put("swing.boldMetal", Boolean.FALSE);
				createAndShowGUI();
			}
		});

		// Start a publisher!
                // TODO: restore this (and correct the dependencies) to actually run this tool.
		/*Publisher p = new Publisher("/sensor/encoder");
		while (true) {
			p.publish(new EncoderMeasurement(41, 42));
			try {
				Thread.sleep(2000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}*/
	}
}
