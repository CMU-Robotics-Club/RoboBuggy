import processing.serial.*;

Serial port;
boolean foundPort = false;

void setup() {
  String[] ports = Serial.list();
  byte[] buffer = new byte[50];
  
  println("Testing Arduino Serial");
  for (int i = 0; i < ports.length; i++) {
      try {
          println("Testing port: " + ports[i]);
          port = new Serial(this, ports[i], 9600);
          
          port.readBytesUntil((byte)0xfc,buffer);
          
          if (buffer != null) {
            while(true) {
              if (port.available() > 0) {
                  if ((byte)port.read() <= 0x5) {
                      println("Connected to port: " + ports[i]);
                      foundPort = true;
                      return;
                  } else { break; }
              }
              delay(10);
            }
          }
          
          port.stop();
      } catch(Exception e) {
          
      }
  }
  println("Unable to find valid port");
  exit();
}

void draw() {
    while(port.available() > 0) {
        char input = (char)port.read();
        
        if (input == (byte)'\n') print(input);
        else print(hex(input, 2));
    }
}
  
