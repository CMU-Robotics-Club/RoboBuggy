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
import struct


INPUTUPPERBOUND = 60

# kill the main thread on sigint
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

screen_copy = None

#Create a dictionary of message headers
# startOfValues is the comment before start of values for dict
# takeout is what needs to be taken out from the beginning of the values in dict
# if it is in the rbsm_config.txt file
def create_bit_to_str(start_of_values, takeout=""):
    dict = {}
    settings_file = None
    try:
        settings_file = open("../../real_time/rbsm_config.txt")
    except:
        print("Error! Unable to find rbsm_headers.txt\n")
        sys.exit(1)
    # when the correct headers found
    headers_found = False
    # when to stop looking for headers
    end_headers = False
    # go through each line to find the message headers
    for line_num, line in enumerate(settings_file):
        # to give right name takes out RBSM_MID_ if that is at beginning cause "RBSM_MID_" unneeded
        if(line[0:len(takeout)] == takeout):
            line = line[len(takeout):]
        # key is in the part after , and value is before
        if(headers_found):
            definition = line.split(", ")
            # if there are not enough values or too many, then line is assumed to be
            # end of the parts with message headers
            if(len(definition)!=2):
                end_headers = True
                break
            # sets the key to equal the message header
            dict[int(definition[1])] = definition[0]
        elif(line[0:len(start_of_values)+3]=="// " + start_of_values):
            # the string that is found is what symbolizes when message headers begin on
            # next line
            headers_found = True
        # when the headers end do not care about anything else so breaks
        if(end_headers):
            break
    return dict

mid_to_str = create_bit_to_str("RBSM Headers", "RBSM_MID_")
eid_to_str = create_bit_to_str("Error Message Bits", "RBSM_EID_")

def redraw(state):
    screen = state["screen"]
    message_cache = state["message_cache"]
    command_line = state["command_line"]
    status_line = state["status_line"]
    
    (max_y, max_x) = screen.getmaxyx()
    row_id = 2

    if status_line == 'Locked.':
        for mid in message_cache.keys():
            # clear line and write new data
            # screen.addstr(row_id, 2, " " * (max_x - 4))
            s = "{:20} {:>10} {:>20}".format(mid_to_str.get(mid, "MID {}".format(mid)),
                                                                            message_cache[mid]["data"],
                                                                            message_cache[mid]["update_time"])
            screen.addstr(row_id, 2, s)

            row_id = row_id + 1
    
        #adding space and then error
        screen.addstr(row_id, 2, "")
        screen.addstr(row_id+1, 2, "--------ERRORS---------")
        # since last steps went through 2 rows adding 2 to row_id
        row_id = row_id + 2
        # finds which value is error
        error = 254
        for key in mid_to_str.keys():
            if(mid_to_str[key]=="ERROR"):
                error = key
        # check if key is even in message_cache
        if(error in message_cache.keys()):
            error_message = message_cache[error]["data"]
            # max bit that has associated error
            maxBit = max(eid_to_str.keys())
            for bit in xrange(maxBit+1):
                if(error_message%2==1 and bit in eid_to_str.keys()):
                    #screen.addstr(row_id, 2, eid_to_str[bit])
                    wipeScreenAndPost(screen, eid_to_str[bit], row_id)
                    row_id = row_id + 1
                # can eliminate previous bit by diving by 2
                error_message = error_message/2
        # adding -- so that if there is nothing between error and this then no error
        wipeScreenAndPost(screen, "------------------------", row_id)
            #screen.addstr(row_id, 2, "------------------------")
        # to make sure row below BottomLine clean:
        wipeScreenAndPost(screen, "", row_id+1)


    # update status line
    screen.addstr(max_y-3, 2, "status: {:<15}".format(status_line))

    # clear command line an write the current one
    screen.addstr(max_y-2, 2, " " * (max_x - 2 - 6))
    screen.addstr(max_y-2, 2, "send? {}".format(command_line))


    screen.refresh()

def wipeScreenAndPost(screen, str, pos):
    # long empty space to wipe line clean
    screen.addstr(pos, 2, "                                                       ")
    screen.addstr(pos, 2, str)

def rbsm_worker(state):
    message_cache = state["message_cache"]
    rbsm_endpoint = state["rbsm_endpoint"]
    
    f = open('bytes from arduino.txt', 'wt') #delete

    while(1):
        new_message = rbsm_endpoint.read()
        if(new_message["status"] == "locked"):
            # update status
            state["status_line"] = "Locked."

            if (new_message["id"] == 0):
                # print("Found encoder")
                pass

            # save new data
            message_cache.update([
                ( new_message["id"], 
                    {"data": new_message["data"], "update_time": time.time()} ),
            ])
        
            if new_message["id"] == 254:
                f.write(str(new_message['data']))
                f.write("\n")

            # update remotely
            # redraw(state)
        else:
            state["status_line"] = "Unlocked!"
    f.close() 

    return None


def command_worker(state):
    screen = state["screen"]
    with open('other.txt', 'at') as f:
                f.write('alive\n')

    while(True):
        new_char = screen.getch()

        if (new_char == curses.KEY_RIGHT or 
            new_char == curses.KEY_LEFT or
            new_char == curses.KEY_UP or
            new_char == curses.KEY_DOWN):
            continue

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
        elif(new_char < 128) and len(state["command_line"]) < INPUTUPPERBOUND:
            state["command_line"] += chr(new_char)

        # redraw(state)

    return None


def command_handler(rbsm_endpoint, command_line):
    error = False
    command_line = command_line.strip()
    command_parts = command_line.split(" ")
    with open('help help.txt', 'wt') as f:
        f.write(repr(command_parts))

        try:
            message_id = int(command_parts[0])
            message_data = int(command_parts[1])
            f.write('not endpoint yet\n')
            rbsm_endpoint.send(message_id, message_data)
            f.write('past endpoint\n')
        except:
            error = True

    return error


def console_drawer(state):
    while(True):
        time.sleep(0.05)
        redraw(state)

def main():
    if(len(sys.argv) < 2):
        print ("You didn't provide enough arguments. Please run with:")
        print ("%s /dev/tty.something" % (sys.argv[0]))
        sys.exit()
    
    state = {
        "message_cache": {},
        "command_line": "",
        "status_line": "Starting...",
        "screen": None,
        "rbsm_endpoint": None,
    }
    screen = state["screen"]


    # setup incomming messages
    try:
        rbsm_endpoint = rbsm_lib.RBSerialMessage(sys.argv[1])
    except:
        print("Probably could not find the specified USB device.")
        print('Please find the correct /dev/tty.* file and try again.')
        return
    
    state["rbsm_endpoint"] = rbsm_endpoint
    rbsm_thread = threading.Thread(target=rbsm_worker, args=(state,))
    rbsm_thread.daemon = True

    # setup reading commands
    command_thread = threading.Thread(target=command_worker, args=(state,))
    command_thread.daemon = True

    #redraw thread
    drawer = threading.Thread(target=console_drawer, args=(state,))
    drawer.daemon = True

    
    # setup program window
    screen = curses.initscr()
    
    curses.noecho()
    screen.clear()
    screen.keypad(1)
    screen.border(0)
    screen.addstr(1, 2, '{:20} {:>10} {:>20}'.format("Message", 'Data', 'Time'))
    state["screen"] = screen
    # redraw(state)

    screenCopy = screen

    rbsm_thread.start()
    command_thread.start()
    drawer.start()

    rbsm_thread.join()
    command_thread.join()
    drawer.join()

    clean_up()


@atexit.register
def clean_up():
    try:
        curses.echo()
        curses.nocbreak()
        screenCopy.keypad(0)
        curses.endwin()
    except:
        pass


if __name__ == "__main__":
    main()
