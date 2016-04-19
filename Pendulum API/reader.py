#!/usr/bin/python

import serial
import signal
import sys

s = None

def exit_handler(signum, frame):
    global s
    s.close()

def main():
    global s

    s = serial.Serial()
    s.port = "/dev/pts/7"
    s.baudrate = 9600
    s.open()


    while(True):
        byte = s.read(1)
        print hex(ord(byte))

if __name__ == "__main__":
    signal.signal(signal.SIGINT, exit_handler)
    main()
    

