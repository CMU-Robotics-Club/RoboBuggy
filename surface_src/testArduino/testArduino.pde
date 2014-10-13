import processing.serial.*;

Serial port;
boolean foundPort = false;

void setup() {
  port = new Serial(this, "COM13",9600);
}

void draw() {
    while(port.available() > 0) {
        char input = (char)port.read();
        
        if (input == (byte)'\n') print(input);
        else print(hex(input, 2));
    }
}
  
