import atexit
import curses
import curses.textpad
import time


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


def redraw(screen, message_cache, command_line):
  (max_y, max_x) = screen.getmaxyx()
  row_id = 2
  for mid in message_cache.keys():
    staleness = message_cache[mid]["update_time"] - time.time()
    screen.addstr(row_id, 2, "                                                                 ")
    screen.addstr(row_id, 2, "%s %s %s" % (mid_to_str[mid],
                                           message_cache[mid]["data"],
                                           staleness) )
    row_id = row_id + 1

  screen.addstr(max_y-2, 2, "                                                                 ")
  screen.addstr(max_y-2, 2, "send? %s" % command_line)

  screen.refresh()


def main():
  message_cache = {}
  command_line = "asdfasdf"
  screen = curses.initscr()
  # screen.nodelay(1)
  curses.noecho()
  screen.clear()
  screen.border(0)
  screen.addstr(1, 2, "Message Data Staleness(s)")

  while(1):
    new_char = screen.getch()
    if(new_char == 8 or new_char == 127):
      command_line = command_line[0:-1]
    elif(new_char == 10 or new_char == 13):
      command_line = ""
    elif(new_char < 128):
      command_line = command_line + chr(new_char)
    # command_line = command_line + str(new_char)

    message_cache.update([
      (10, {"data": 2304, "update_time": time.time()}),
      (11, {"data": 2304, "update_time": time.time()})
    ])

    redraw(screen, message_cache, command_line)


@atexit.register
def clean_up():
  curses.endwin()


if __name__ == "__main__":
  main()
