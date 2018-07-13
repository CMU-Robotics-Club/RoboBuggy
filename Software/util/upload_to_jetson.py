#!/usr/bin/env python2

# Python 2.7 script for uploading current changes to a Jetson
# Make sure to run this file from the util/ directory, otherwise paths will get messed up
# Note that this assumes:
#   - You've already configured the target Jetson to be on the correct branch
#   - You aren't using the ubuntu version of ping, you're using the macos version (ping -t is timeout on mac)

import argparse
import subprocess

jetson_names = {
    "Ackermann": "ackermann",
    "Kalman": "kalman",
    "PID": "pid",
    "Stanley": "stanley",
    "TX1": "-tx1"
}

parser = argparse.ArgumentParser(description="Upload current changes to a Jetson")
parser.add_argument("jetson_name", 
                    type=str, 
                    help="Jetson name",
                    choices=list(jetson_names)
                    ) 
args = parser.parse_args()

def main():
    jetson_hostname = "robobuggy{}.wv.cc.cmu.edu".format(jetson_names[args.jetson_name])

    # check if we can ping it at all
    print "Checking if {} is awake...".format(args.jetson_name)
    errno = subprocess.call("ping -t 1 {} > /dev/null".format(jetson_hostname), shell=True)
    if errno != 0:
        print "Couldn't ping host! Exiting!"
        exit(1)

    # get acknowledgement the jetson is ready
    print "Have you switched to your branch on the Jetson? ([y]/n)"
    ack = raw_input()
    if ack != 'y' and ack != "":
        print "Didn't answer \'y\', exiting!"
        exit(1)

    # get acknowledgement all files desired are there
    print "Have you added any untracked files desired? ([y]/n)"
    ack = raw_input()
    if ack != 'y' and ack != "":
        print "Didn't answer \'y\', exiting!"
        exit(1)
    
    # get git output
    print "Getting list of files modified / added"
    git_output = subprocess.check_output("git status --porcelain", shell=True)
    lines = git_output.splitlines()
    files_to_transfer = []
    for line in lines:
        if "A" in line[0:2] or "M" in line[0:2]:
            files_to_transfer.append(line[3:])

    argstring = ""
    for filename in files_to_transfer:
        # tar goes off files from the home dir, need to add "robobuggy" to the front
        argstring += "RoboBuggy/" + filename + " "

    subprocess.call("cd ../../..; tar cvzf - {0} | ssh nvidia@{1} tar xzf -".format(argstring, jetson_hostname), shell=True)


if __name__ == "__main__":
    main()