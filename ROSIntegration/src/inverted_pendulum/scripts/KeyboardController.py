#!/usr/bin/env python

import rospy
from inverted_pendulum.msg import Cmd

magnitude = 10


class keyboard_controller:
    def __init__(self):
        self.keymap = {}
        #support for wsad keys
        self.keymap['w'] = "UP"	
        self.keymap['s'] = "DOWN"	
        self.keymap['a'] = "LEFT"	
        self.keymap['d'] = "RIGHT"	

        #support for arrow keys
        self.keymap['\x41'] = "UP"
        self.keymap['\x42'] = "DOWN"
        self.keymap['\x44'] = "LEFT"
        self.keymap['\x43'] = "RIGHT"

    def get_action(self):
        import os, signal
        while 1:
            c = self.getc()
            #handle special keys
            if(c == '\x03'): #sigint
                rospy.loginfo("catching sigint")
                os.kill(os.getpid(), signal.SIGKILL)
            if(c == '\x1a'): #send to background
                os.kill(os.getpid(), signal.SIGTSTP)

            if(c == '\x1b'): #escape code, could be arrow keys
                c2 = getc()
                if(c2 == '\x5b'): #confirm
                    c = getc()

            if(c in self.keymap):
                return self.keymap[c]
    def getc(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

if __name__ == "__main__":
    rospy.init_node('KeyboardController')
    cmd_pub = rospy.Publisher('/cmd', Cmd, queue_size=1)
    rate = rospy.Rate(10)
    kb = keyboard_controller()

    while not rospy.is_shutdown():
        cmd = Cmd()
        action = kb.get_action()
        
        if(action == "LEFT"):
            cmd.cmd = -magnitude
        if(action == "RIGHT"):
            cmd.cmd = magnitude
        
        cmd_pub.publish(cmd)
    

