import serial
import struct
import sys


RBSM_MID_ENC_TICKS_LAST_H = 0
RBSM_MID_ENC_TICKS_LAST_L = 1
RBSM_MID_ENC_TICKS_RESET_H = 2
RBSM_MID_ENC_TICKS_RESET_L = 3
RBSM_MID_ENC_TIMESTAMP_H = 4
RBSM_MID_ENC_TIMESTAMP_L = 5
RBSM_MID_RESERVED = 252 # 0xFC, message head
RBSM_MID_DEVICE_ID = 255


def byte_read(serial):
  asdf = serial.read(1)
  print asdf
  ord(asdf)


def main():
  if(len(sys.argv) < 2):
    print "I need more args. Usage:"
    print "%s /dev/tty.usbserial???" % (sys.argv[0])

  # get serial connection
  encoder_serial = serial.Serial(sys.argv[1], 9600)

  # read data
  encoder_ticks_last = 0
  encoder_ticks_reset = 0
  encoder_timestamp = 0

  # calculated data
  encoder_velocity = 0

  last_read = byte_read(encoder_serial)
  while(True):
    if(last_read == RBSM_MID_RESERVED):
      while(True):
        last_read = byte_read(encoder_serial)
        if(not(last_read == RBSM_MID_RESERVED)): # guard for real header
          m_id = last_read
          m_payload_h = byte_read(encoder_serial)
          if(m_payload_h == RBSM_MID_RESERVED): byte_read(encoder_serial)
          m_payload_l = byte_read(encoder_serial)
          if(m_payload_l == RBSM_MID_RESERVED): byte_read(encoder_serial)
          m_payload = (m_payload_h << 8) & m_payload_l;

        print "Got ID %d with payload %d." % (m_id, m_payload)



if __name__ == "__main__":
  main()
