package com.roboclub.robobuggy.ros.tools;

/** Disaply for seeing the contents of messages as they are broadcasted **/

import java.awt.BorderLayout; //for layout managers and more
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener; //for action events

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

import com.roboclub.robobuggy.ros.Message;
import com.roboclub.robobuggy.ros.MessageListener;
import com.roboclub.robobuggy.ros.Subscriber;

public class MessageListenerGui extends JPanel implements ActionListener {
	private String newline = "\n";
	private JTextField filterTextField;
	private DefaultTableModel model;

	private Subscriber allMessages;

	private class Worker extends SwingWorker {

		@Override
		protected Object doInBackground() throws Exception {
			System.out.println(";lkjasdl;kjasdf");
			Subscriber s = new Subscriber("/sensor/encoder",
					new MessageListener() {
						@Override
						public void actionPerformed(Message m) {
							System.out.println("IT RAN I AM SO HAPPY");
						}
					});
			while (true) {
				// TODO fix this.
				Thread.sleep(100);
			}
		}
	}

	public MessageListenerGui() {
		setLayout(new BorderLayout());

		filterTextField = new JTextField(100);
		filterTextField.addActionListener(this);

		// Create some labels for the fields.
		JLabel textFieldLabel = new JLabel("Filter:");
		textFieldLabel.setLabelFor(filterTextField);

		// Lay out the text controls and the labels.
		JPanel controlPane = new JPanel();
		GridBagLayout gridbag = new GridBagLayout();
		GridBagConstraints c = new GridBagConstraints();
		controlPane.setLayout(gridbag);
		c.gridwidth = GridBagConstraints.REMAINDER; // last
		c.anchor = GridBagConstraints.WEST;
		c.weightx = 1.0;
		controlPane.add(filterTextField, c);
		controlPane.setBorder(BorderFactory.createCompoundBorder(BorderFactory
				.createTitledBorder("Type a prefix then press enter"),
				BorderFactory.createEmptyBorder(5, 5, 5, 5)));

		// Set up table for bottom pane
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

		// Start a thread to service updates about messages
		/*
		 * EventQueue.invokeLater(new Runnable() { Subscriber s;
		 *
		 * @Override public void run() { // TODO Auto-generated method stub
		 * System.out.println(";lkjasdl;kjasdf"); s = new
		 * Subscriber("/sensor/encoder", new MessageListener() {
		 *
		 * @Override public void actionPerformed(Message m) {
		 * System.out.println("received!"); String[] tmp = { "wasd", "wasd" };
		 * model.addRow(tmp); } }); } });
		 */
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		System.out.println("Enter was hit!");
		String input = filterTextField.getText();
		System.out.println(input);
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
		frame.add(new MessageListenerGui());

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

	public static void main(String[] args) {
		// Schedule a job for the event dispatching thread:
		// creating and showing this application's GUI.
		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				// Turn off metal's use of bold fonts
				UIManager.put("swing.boldMetal", Boolean.FALSE);
				createAndShowGUI();
			}
		});
	}
}