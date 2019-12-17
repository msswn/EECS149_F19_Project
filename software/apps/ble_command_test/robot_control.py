#!/usr/bin/env python3
import struct
import time
import keyboard
from getpass import getpass
from bluepy.btle import Peripheral, DefaultDelegate
import argparse


parser = argparse.ArgumentParser(description='Print advertisement data from a BLE device')
parser.add_argument('addr', metavar='A', type=str, help='Address of the form XX:XX:XX:XX:XX:XX')
args = parser.parse_args()
addr = args.addr.lower()
if len(addr) != 17:
    raise ValueError("Invalid address supplied")

SERVICE_UUID = "4607eda0-f65e-4d59-a9ff-84420d87a4ca"
CHAR_UUIDS = ["4607eda3-f65e-4d59-a9ff-84420d87a4ca", "4607eda4-f65e-4d59-a9ff-84420d87a4ca", "4607eda5-f65e-4d59-a9ff-84420d87a4ca", "4607eda6-f65e-4d59-a9ff-84420d87a4ca"] # TODO: add your characteristics
# LED_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
# LED_CHAR_UUID    = "32e61090-2b22-4db5-a914-43ce41986c70"

# buckler = Peripheral(addr)
# led_sv = buckler.getServiceByUUID(LED_SERVICE_UUID)
# led_ch = led_sv.getCharacteristics(LED_CHAR_UUID)[0]

class RobotController():
    def __init__(self, address):

        self.robot = Peripheral(addr)

        print("connected")

        # keep state for keypresses
        self.pressed = {"up": False, "down": False, "right": False, "left": False}
        # TODO get service from robot
        # TODO get characteristic handles from service/robot
        # TODO enable notifications if using notifications
        sv = self.robot.getServiceByUUID(SERVICE_UUID)
        self.ch_forward = sv.getCharacteristics(CHAR_UUIDS[0])[0]
        self.ch_backward = sv.getCharacteristics(CHAR_UUIDS[1])[0]
        self.ch_left = sv.getCharacteristics(CHAR_UUIDS[2])[0]
        self.ch_right = sv.getCharacteristics(CHAR_UUIDS[3])[0]

        keyboard.hook(self.on_key_event)

    def on_key_event(self, event):
        # print key name
        print(event.name)
        # if a key unrelated to direction keys is pressed, ignore
        if event.name not in self.pressed: return
        # if a key is pressed down
        if event.event_type == keyboard.KEY_DOWN:
            # if that key is already pressed down, ignore
            if self.pressed[event.name]: return
            # set state of key to pressed
            self.pressed[event.name] = True
            # TODO write to characteristic to change direction
            if event.name == "up":
                #print("writing forward")
                self.ch_forward.write(b'\x01')
            elif event.name == "down":
                #print("writing back")
                self.ch_backward.write(b'\x01')
            elif event.name == "right":
               # print("writing right")
                self.ch_right.write(b'\x01')
            elif event.name == "left":
                #print("writing left")
                self.ch_left.write(b'\x01')
        else:
            # set state of key to released
            self.pressed[event.name] = False
            # TODO write to characteristic to stop moving in this direction
            print("Stopping")
            self.ch_forward.write(b'\x00')
            self.ch_backward.write(b'\x00')
            self.ch_right.write(b'\x00')
            self.ch_left.write(b'\x00')


    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc_value, traceback):
        self.robot.disconnect()

with RobotController(addr) as robot:
    getpass('Use arrow keys to control robot')
