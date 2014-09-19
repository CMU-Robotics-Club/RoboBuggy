package main;

public class GpsPanel extends SerialPanel {
	private static final long serialVersionUID = 1399590586061060311L;
	private static final char[] HEADER = {'#', 'A', 'C', 'G'};
	private static final int HEADER_LEN = 4;

	public GpsPanel() {
		super("GPS", 57600, HEADER, HEADER_LEN);
		
	}
}