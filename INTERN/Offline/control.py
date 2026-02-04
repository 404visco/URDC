import collections
import cv2
import time
import numpy as np

from collections import abc
collections.MutableMapping = abc.MutableMapping

from pymavlink import mavutil
from dronekit import VehicleMode,connect

#Connect Drone
def connect_vehicle(vehicle):
    vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
    return vehicle

#Ubah Mode
def change_mode(vehicle, flight_mode):
    vehicle.mode =VehicleMode(flight_mode)
    vehicle.wait_for_mode(flight_mode)
    if not vehicle.mode.name == flight_mode:
        print('Failed to change mode')
    print(f'Flight mode changed to {flight_mode}')

def arm(vehicle):
    print('Arming')
    vehicle.arm(wait=True)
    while not vehicle.armed:
        print('Wait...')
        time.sleep(1)
    print('Armed')

def disarm(vehicle)
