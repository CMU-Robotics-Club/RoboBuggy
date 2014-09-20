package main;

public class ArduinoPanel extends SerialPanel {
	private static final long serialVersionUID = -929040896215455343L;
	private static final char[] HEADER = {'a'};
	private static final int HEADER_LEN = 1;

	public ArduinoPanel(String port_name, int baud_rate) throws Exception {
		super("ARDUINO", 9600, HEADER, HEADER_LEN);
		
	}
}