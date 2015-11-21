import serial;
import sys
import struct


class RBSerialMessage:

  def __init__(self, device_path):
    self.RBSM_FOOTER = "\n" # 0x0A
    self.RBSM_BAUD_RATE = 115200
    self.RBSM_PACKET_LENGTH = 6
    # packet format: unsigned byte, big-endian signed int, char
    self.RBSM_PACKET_FORMAT = ">Bic"

    self.device_path = device_path
    self.port = serial.Serial(device_path, self.RBSM_BAUD_RATE)

    self.stream_lock = False

  def read(self):
    # if we don't have a stream lock read until we find a footer
    if(self.stream_lock == False):
      possible_footer = self.port.read(1)
      while(possible_footer != self.RBSM_FOOTER):
        possible_footer = self.port.read(1)
      self.stream_lock = True

    # capture a packet length
    message_buffer = ""
    message_unpacked = None
    message_id = None
    message_data = None

    # read what we think is a full packet
    for i in range(self.RBSM_PACKET_LENGTH):
      message_buffer += self.port.read(1)

    # unpack the packet
    message_unpacked = struct.unpack_from(self.RBSM_PACKET_FORMAT, message_buffer)

    # confirm properly formed packet
    if(message_unpacked[2] == self.RBSM_FOOTER):
      return {"id": message_unpacked[0],
              "data": message_unpacked[1],
              "status": "locked"}

    # but unlock the stream if we can't find a footer
    else:
      self.stream_lock = False
      return {"id": 0, "data": 0, "status": "unlocked"}

  def send(self, message_id, message_data):
    message_buffer = struct.pack(self.RBSM_PACKET_FORMAT,
                                 message_id,
                                 message_data,
                                 self.RBSM_FOOTER)
    print repr(message_buffer)
    print len(message_buffer)
    self.port.write(message_buffer)
    self.port.flush()


if __name__ == "__main__":
  import time
  print("waiting for reset...")
  time.sleep(10)
  print("sending test messages:")
  rbsm_endpoint = RBSerialMessage(sys.argv[1])
  # send some test RBSM_MID_MEGA_STEER_ANGLE messages
  rbsm_endpoint.send(20, 0)
  rbsm_endpoint.send(20, -250)
  rbsm_endpoint.send(20, 250)
  rbsm_endpoint.send(20, 0)
  print("send messages successfully.")
  
  # then listen for messages forever
  print("here are some messages I'm seeing:")
  # while(1):
  #   message = rbsm_endpoint.read()
  #   print (message)
