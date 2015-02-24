#!/usr/bin/env python

'''
Copyright 2012 the original author or authors.
See the NOTICE file distributed with this work for additional
information regarding copyright ownership.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
'''

import serial
from curses import ascii
import rospy
from colorama.ansi import Fore
import psutil
from time import sleep
from os import path


class OdroidShow(object):
    def __init__(self, path):
        self.device = serial.Serial(path, 500000)

    def clear(self):
        self.device.write(chr(ascii.ESC) + "[2J")

    def display(self, format_str, **kwargs):
        msg = format_str.format(**kwargs)
        msg = msg.replace('\n', '\r\n')
        self.device.write(msg)

    def set_cursor_to_home(self):
        self.device.write(chr(ascii.ESC) + "[H")


class OdroidShowNode(object):
    status_chars = r"\|/-"

    def __init__(self, path):
        self.device = OdroidShow(path)
        self.next_status_char = 0

    def run(self):
        self.device.clear()
        while not rospy.is_shutdown():
            rospy.sleep(1.0)
            self.device.set_cursor_to_home()
            self.print_spinner()
            self.print_wifi_status()

    def print_spinner(self):
        self.next_status_char += 1
        self.next_status_char %= 4
        self.device.display(Fore.YELLOW)
        self.device.display(OdroidShowNode.status_chars[self.next_status_char])
        self.device.display("\n")

    def print_wifi_status(self):
        self.device.display("WiFi: ")
        if self.wifi_is_up():
            # Print two extra spaces, so that it's the same width as "down"
            self.device.display(Fore.GREEN + 'UP  ')
        else:
            self.device.display(Fore.RED + 'DOWN')

    def wifi_is_up(self):
        for p in psutil.process_iter():
            if 'hostapd' in p.name:
                return True
        return False

if __name__ == '__main__':
    while not path.exists("/dev/odroid_show"):
        sleep(1.0)
    OdroidShowNode("/dev/odroid_show").run()
