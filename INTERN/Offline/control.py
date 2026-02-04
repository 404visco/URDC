import collections
import cv2
import time
import numpy as np

from collections import abc
collections.MutableMapping = abc.MutableMapping

from pymavlink import mavutil
from dronekit import VehicleMode,connect

#Connect Drone
def connect_vehicle(vehicle:object):
    vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
    return vehicle

#Ubah Mode
def change_mode(vehicle:object, flight_mode:str):
    vehicle.mode =VehicleMode(flight_mode)
    vehicle.wait_for_mode(flight_mode)
    if not vehicle.mode.name == flight_mode:
        print('Failed to change mode')
    print(f'Flight mode changed to {flight_mode}')

#Arm
def arm(vehicle: object):
    print('Arming')
    vehicle.arm(wait=True)
    while not vehicle.armed:
        print('Wait...')
        time.sleep(1)
    print('Armed')

#Disarm
def disarm(vehicle:object):
    print('Disarming')
    vehicle.disarm(wait=True)
    print('Disarmed')

#Takeoff
def takeoff(vehicle:object, altitude: float):
    change_mode(vehicle, 'GUIDED') #Ubah ke mode guided dulu
    arm(vehicle) #arm
    print('Takeoff')
    vehicle.simple_takeoff(altitude)
    while True: #Tunggu sampai drone sudah mencapai target altitude
        alt0 = vehicle.location.global_relative_frame.alt #pantau altitude
        if alt0 >= altitude*0.95: #toleransi agar tidak loop selamanya
            break
        time.sleep(1)

