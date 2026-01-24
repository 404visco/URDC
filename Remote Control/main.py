#Setup
import collections
import cv2
import numpy as np

from time import sleep
from collections import abc
collections.MutableMapping = abc.MutableMapping

from dronekit import connect,VehicleMode
from pymavlink import mavutil

#delay
def count(time):
    for i in range(time):
        print(i+1)
        sleep(1)
    print('lesgo!')
count(2)

#Sambung drone
def connect_vehicle():
    vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
    return True

#Change Mode
def change_mode(vehicle, flight_mode):
    vehicle.mode= VehicleMode(flight_mode) #Atur Mode
    vehicle.wait_for_mode(flight_mode) #Nunggu mode berubah
    if vehicle.mode.name == flight_mode:
        print(f"Masuk mode {flight_mode}🫡")
    else:
        print("Gagal ganti mode😥")
