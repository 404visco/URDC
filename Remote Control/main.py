#Setup
import collections
import cv2
import numpy as np

from time import sleep
from collections import abc
collections.MutableMapping = abc.MutableMapping

from dronekit import connect,VehicleMode
from pymavlink import mavutil

def count(time):
    for i in range(time):
        print(i+1)
        sleep(1)
    print('lesgo!')
count(2)
#Arm