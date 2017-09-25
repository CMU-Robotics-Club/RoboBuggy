import serial 

ser = serial.Serial('COM4', 9600)
message = 0xFC01DEAD
ser.write(hex(message))
print "Wrote to Arduino"

ret_pckt = ser.read(4)
print "returned from arduino"
print ret_pckt
