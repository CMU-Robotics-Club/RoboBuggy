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
  10: "MEGA_STEER_ANGLE",
  11: "MEGA_BRAKE_STATE",
  252: "RESERVED",
  254: "ERROR",
  255: "DEVICE_ID"
}


def redraw(state):
  screen = state["screen"]
  message_cache = state["message_cache"]
  command_line = state["command_line"]
  
  (max_y, max_x) = screen.getmaxyx()
  row_id = 2
  for mid in message_cache.keys():
    screen.addstr(row_id, 2, "                                                                 ")
    screen.addstr(row_id, 2, "%s %s %s" % (mid_to_str[mid],
                                           message_cache[mid]["data"],
                                           message_cache[mid]["update_time"]) )
    row_id = row_id + 1

  screen.addstr(max_y-2, 2, "                                                                 ")
  screen.addstr(max_y-2, 2, "send? %s" % command_line)

  screen.refresh()


def rbsm_worker(state):
  message_cache = state["message_cache"]
  rbsm_endpoint = state["rbsm_endpoint"]

  while(1):
    new_message = rbsm_endpoint.read()
    if(new_message["status"] == "locked"):
      # save new data
      message_cache.update([
        ( new_message["id"], 
          {"data": new_message["data"], "update_time": time.time()} ),
      ])
      # update remotely
      redraw(state)

  return None


def command_worker(state):
  screen = state["screen"]

  while(1):
    new_char = screen.getch()
    if(new_char == 8 or new_char == 127):
      state["command_line"] = state["command_line"][0:-1]
    elif(new_char == 10 or new_char == 13):
      state["command_line"] = ""
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
  except:
    error = True

  try:
    message_data = int(command_parts[1])
  except:
    error = True

  rbsm_endpoint.send(message_id, message_data)

  return error



def main():
  state = {
    "message_cache": {},
    "command_line": "",
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
  curses.endwin()


if __name__ == "__main__":
  main()
