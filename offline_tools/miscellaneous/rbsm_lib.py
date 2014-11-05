import serial;
import sys

class RBSerialMessage:

  def __init__(self, device_path):
    self.RBSM_FOOTER = 0x0A
    self.RBSM_BAUD_RATE = 9600
    self.RBSM_PACKET_LENGTH = 6

    self.device_path = device_path
    self.port = serial.Serial(device_path, self.RBSM_BAUD_RATE)

    self.stream_lock = False

  def read(self):
    # if we don't have a stream lock read until we find a footer
    if(self.stream_lock == False):
      possible_footer = ord(self.port.read(1))
      while(possible_footer != self.RBSM_FOOTER):
        possible_footer = ord(self.port.read(1))
      self.stream_lock = True

    # capture a packet length
    message_buffer = []
    message_id = None
    message_data = None
    for i in range(self.RBSM_PACKET_LENGTH):
      message_buffer.append(ord(self.port.read(1)))

    # confirm correct packet structure
    if(message_buffer[self.RBSM_PACKET_LENGTH - 1] == self.RBSM_FOOTER):
      message_id = message_buffer[0]
      # message_data = message_buffer[1:5]
      message_data = ((message_buffer[1] << 24) +
                      (message_buffer[2] << 16) +
                      (message_buffer[3] << 8) +
                      (message_buffer[4])) & 0xFFFFFFFF
      # for j in range(4):
      #   message_data = (message_data << 8) | message_buffer[4-j]
      return {"id": message_id, "data": message_data, "status": "locked"}
    else:
      self.stream_lock = False
      return {"id": 0, "data": 0, "status": "unlocked"}

  def send(self, message_id, message_data):
    message_buffer = []
    # add header
    message_buffer.append(message_id & 0xFF)
    # add data in big endian byte-order
    message_buffer.append((message_data >> 24) & 0xFF)
    message_buffer.append((message_data >> 16) & 0xFF)
    message_buffer.append((message_data >> 8) & 0xFF)
    message_buffer.append((message_data) & 0xFF)
    # add footer
    message_buffer.append(self.RBSM_FOOTER)

    self.port.write(message_buffer)

if __name__ == "__main__":
  rbsm_endpoint = RBSerialMessage(sys.argv[1])
  while(1):
    message = rbsm_endpoint.read()
    print message
    # print "%d: %d (%s)\r\n" % (message["id"], message["data"], message["status"])
