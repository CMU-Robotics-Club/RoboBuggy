# rbsm_console.py
# Author: Ian Hartwig (ihartwig)
# 
# Python shell program for reading and writing robobuggy serial messages.
# Features an ncurses UI that displays the latest data for all message types
# received as well as accepting user input to send a command. The program spawns
# 2 daemon threads in the background to handle message receiving and keyboard
# events. The display is updated when either action happens
# 
# To start:
# python rbsm_console.py /dev/tty.something
# 
# To send a command from the console:
# send? message_id message_data
# 

import atexit
import curses
import rbsm_lib
import sys
import threading
import time

# kill the main thread on sigint
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

mid_to_str = {
  0: "ENC_TICKS_LAST",
  1: "ENC_TICKS_RESET",
  2: "ENC_TIMESTAMP",
  20: "MEGA_STEER_ANGLE",
  21: "MEGA_BRAKE_STATE",
  22: "MEGA_AUTON_STATE",
  23: "MEGA_BATTERY_LEVEL",
  252: "RESERVED",
  254: "ERROR",
  255: "DEVICE_ID"
}


def redraw(state):
  screen = state["screen"]
  message_cache = state["message_cache"]
  command_line = state["command_line"]
  status_line = state["status_line"]
  
  (max_y, max_x) = screen.getmaxyx()
  row_id = 2

  try:
    for mid in message_cache.keys():
      # clear line and write new data
      screen.addstr(row_id, 2, " " * (max_x - 2))
      screen.addstr(row_id, 2, "%s %s %s" % (mid_to_str.get(mid, "MID %d"%(mid)),
                                             message_cache[mid]["data"],
                                             message_cache[mid]["update_time"]))
      row_id = row_id + 1
  except:
    pass

  # update status line
  screen.addstr(max_y-3, 2, " " * (max_x - 2 - 8))
  screen.addstr(max_y-3, 2, "status: %s" % status_line)

  # clear command line an write the current one
  screen.addstr(max_y-2, 2, " " * (max_x - 2 - 6))
  screen.addstr(max_y-2, 2, "send? %s" % command_line)

  screen.refresh()


def rbsm_worker(state):
  message_cache = state["message_cache"]
  rbsm_endpoint = state["rbsm_endpoint"]

  while(1):
    new_message = rbsm_endpoint.read()
    if(new_message["status"] == "locked"):
      # update status
      state["status_line"] = "Locked."
      # save new data
      message_cache.update([
        ( new_message["id"], 
          {"data": new_message["data"], "update_time": time.time()} ),
      ])
      # update remotely
      redraw(state)
    else:
      state["status_line"] = "Unlocked!"

  return None


def command_worker(state):
  screen = state["screen"]

  while(1):
    new_char = screen.getch()

    # backspace
    if(new_char == 8 or new_char == 127):
      state["command_line"] = state["command_line"][0:-1]

    # enter
    elif(new_char == 10 or new_char == 13):
      # try to run the command
      error = command_handler(state["rbsm_endpoint"], state["command_line"])
      if(error):
        state["status_line"] = "Error sending message!"
      # clear visible command line
      state["command_line"] = ""

    # standard ascii characters
    elif(new_char < 128):
      state["command_line"] = state["command_line"] + chr(new_char)

    redraw(state)

  return None


def command_handler(rbsm_endpoint, command_line):
  error = False
  command_line = command_line.strip()
  command_parts = command_line.split(" ")

  try:
    message_id = int(command_parts[0])
    message_data = int(command_parts[1])
    rbsm_endpoint.send(message_id, message_data)
  except:
    error = True

  return error



def main():
  if(len(sys.argv) < 2):
    print "You didn't provide enough arguments. Please run with:"
    print "%s /dev/tty.something" % (sys.argv[0])
    sys.exit()

  state = {
    "message_cache": {},
    "command_line": "",
    "status_line": "Starting...",
    "screen": None,
    "rbsm_endpoint": None,
  }
  screen = state["screen"]

  # setup program window
  screen = curses.initscr()
  curses.noecho()
  screen.clear()
  screen.border(0)
  screen.addstr(1, 2, "Message Data Time")
  state["screen"] = screen
  redraw(state)

  # setup incomming messages
  rbsm_endpoint = rbsm_lib.RBSerialMessage(sys.argv[1])
  state["rbsm_endpoint"] = rbsm_endpoint
  rbsm_thread = threading.Thread(target=rbsm_worker, args=(state,))
  rbsm_thread.daemon = True
  rbsm_thread.start()

  # setup reading commands
  command_thread = threading.Thread(target=command_worker, args=(state,))
  command_thread.daemon = True
  command_thread.start()

  rbsm_thread.join()
  command_thread.join()

  clean_up()


@atexit.register
def clean_up():
  try:
    curses.endwin()
  except:
    pass


if __name__ == "__main__":
  main()
