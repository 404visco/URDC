import collections
import cv2
import time
import numpy as np
import math

from collections import abc
collections.MutableMapping = abc.MutableMapping

from pymavlink import mavutil
from dronekit import VehicleMode,connect,LocationGlobalRelative
from yolo import *
import global_variable as gv

#Connect Drone
def connect_vehicle():
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

#Perintah gerak
def send_ned_velocity(vehicle: object, velocity_x: float, velocity_y: float, velocity_z: float):
    to_send = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE,
        0, 0, 0,
        velocity_x, velocity_y, velocity_z,
        0, 0, 0,
        0, 0        
    )
    
    vehicle.send_mavlink(to_send)

def send_yaw(vehicle: object,
             target_yaw:float,
             arah:int, #1 =Searah jarum jam, -1 berlawanan jarum jam
             v_yaw= 15,
             relative=True):
    to_send= vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        target_yaw,
        v_yaw,
        arah,
        1 if relative else 0,
        0, 0, 0
    )
    vehicle.send_mavlink(to_send)

#Takeoff
def takeoff(vehicle:object, altitude=10.0):
    change_mode(vehicle, 'GUIDED') #Ubah ke mode guided dulu
    arm(vehicle) #arm
    print('Takeoff')
    vehicle.simple_takeoff(altitude)
    while True: #Tunggu sampai drone sudah mencapai target altitude
        alt0 = vehicle.location.global_relative_frame.alt #pantau altitude
        if alt0 >= altitude*0.95: #toleransi agar tidak loop selamanya
            break
        time.sleep(1)

#Landing
def land(vehicle:object):
    print('Landing...')
    change_mode(vehicle, 'LAND')
    print('Landed')

#gerakan
def maju(speed=0.5):
        vx = vy = vz = 0 #awal semua 0
        vx= +speed
        return vx,vy,vz

def mundur(speed=0.5):
        vx=vy=vz=0
        vx= -speed
        return vx,vy,vz

def kanan(speed=0.5):
        vx=vy=vz=0
        vy= +speed
        return vx,vy,vz

def kiri(speed=0.5):
        vx=vy=vz=0
        vy= -speed
        return vx,vy,vz

def f0(vehicle):
    if gv.finger_count==0:
        arm(vehicle)
    elif gv.finger_count==1:
        takeoff(vehicle)

def f1(vehicle):
    vx = vy = vz = 0.0
    if gv.finger_count == 1:
        land(vehicle)
    elif gv.finger_count == 2:
        vx, vy, vz = kanan()
    elif gv.finger_count == 3:
        vx, vy, vz = kiri()
    elif gv.finger_count == 4:
        vx, vy, vz = mundur()
    elif gv.finger_count == 5:
        vx, vy, vz = maju()
    return vx, vy, vz

         