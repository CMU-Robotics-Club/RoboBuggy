package serial;

import java.util.EventListener;

public class SerialEvent implements EventListener {
	private char[] buffer;
	private int len;
	
	public SerialEvent(char[] buffer_, int len_) {
		buffer = buffer_;
		len = len_;
	}
	
	public char[] getBuffer() {
		return this.buffer;
	}
	
	public int getLength() {
		return this.len;
	}
}