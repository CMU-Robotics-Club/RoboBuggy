""" fingerprint.py: 
* Description: This is a python script to create a .h file defining the time 
*   of compilation and who compiled.  Also details whether files had been
*   modified since the last commit
*
*
*   By Sean Buckley
"""


import os
import datetime
import socket #Used for getting hostname
import subprocess

def main():
    print("Now creating fingerprint.h...\n")
    file = open("fingerprint.h", 'w')
    file.write("/* fingerprint.h: A compilation-time created file containing compilation time, \n")
    file.write("closest version of git, and other information of */\n\n")
    time = datetime.datetime.now()
    file.write("#ifndef FINGERPRINT_H\n#define FINGERPRINT_H\n\n")
    file.write("#define FP_COMPDATE \"%02d:%02d:%04d\" //day:month:year\n" %(time.day, time.month, time.year))
    file.write("#define FP_COMPTIME \"%02d:%02d:%02d\" //hour:minute:second\n" %(time.hour, time.minute, time.second))
    file.write("#define FP_HOSTNAME \"%s\"\n" %socket.gethostname().split(".")[0])


    branch = str(subprocess.check_output(["git", "symbolic-ref", "--short", "HEAD"])).replace("\n", "")

    file.write("#define FP_BRANCHNAME \"%s\"\n" %branch)

    result = str(subprocess.check_output(["git", "status"]))

    clean = False

    for line in result.rsplit("\n"):
        if "working directory clean" in line:
            clean = True


    hString = str(subprocess.check_output(["git", "log", "-1", "--pretty=format:'%h'"])).rstrip().replace("'", "")

    file.write("#define FP_HEXCOMMITHASH 0x%s \n" %(hString))
    file.write("#define FP_STRCOMMITHASH \"%s\"\n" %(hString))
    file.write("#define FP_CLEANSTATUS %s\n" %str(clean).lower())
    file.write("\n#endif")


    file.close()
    print("File created.\n")

main()