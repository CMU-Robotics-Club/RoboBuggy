package main;

import javax.swing.JPanel;

public class ArduinoPanel extends SerialPanel {
	private static final long serialVersionUID = -929040896215455343L;

	public ArduinoPanel(String port_name, int baud_rate) throws Exception {
		super(port_name, "ARDUINO", 9600);
		
	}
}