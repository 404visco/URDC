import collections
import cv2
import time
import numpy as np

from collections import abc
collections.MutableMapping = abc.MutableMapping

from pymavlink import mavutil
from dronekit import VehicleMode,connect

def connect_vehicle(vehicle):
    vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
    return vehicle

def change_mode(vehicle, flight_mode):
    