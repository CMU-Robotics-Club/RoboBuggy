
public interface FauxParser {
	
	void parse();
	void qAdd(String input);
	static void setOffset(long offset) {}
}
