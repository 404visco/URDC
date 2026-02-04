import collections
import cv2
import time
import numpy as np
import math

from collections import abc
collections.MutableMapping = abc.MutableMapping

from pymavlink import mavutil
from dronekit import VehicleMode,connect,LocationGlobalRelative

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

#Perintah gerak
def send_ned_velocity(vehicle: object, vx: float, vy:float, vz: float):
    to_send= vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        mavutil.amvlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE,
        0, 0, 0,
        vx, vy, vz,
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
        1 if target_yaw>= 0 else -1,
        1 if relative else 0,
        0, 0, 0
    )
    vehicle.send_mavlink(to_send)

#setup gerak
def jarak(real_koor, NorthSouth, EastWest): #membuat koor m ke radian
    r_earth= 6378137 #radius bumi
    lat_target= NorthSouth/r_earth #radian lat= jarak/radius
    lon_target= EastWest/(r_earth*math.cos(math.radians(real_koor.lat))) # dikali cos(latitude) karena semakin dekat kutub, longitude semakin kecil

    #koor asli ditambah koor plus 10m
    newlat= real_koor.lat + math.degrees(lat_target)
    newlon= real_koor.lon + math.degrees(lon_target) 

    return LocationGlobalRelative(newlat,newlon,real_koor.alt) #koor baru

def gerak(vehicle,jarak):
    current_loc= vehicle.location.global_relative_frame
    arah_drjt =  vehicle.heading
    arah_rad = math.radians(arah_drjt)
    gerak_NorthSouth= jarak * math.cos(arah_rad)
    gerak_EastWest= jarak * math.sin(arah_rad)
    return jarak(current_loc, gerak_NorthSouth, gerak_EastWest)

def get_distance_metres(loc1, loc2):
    dlat = loc2.lat - loc1.lat #jarak latitude
    dlon = loc2.lon - loc1.lon #jarak longitude

    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 111319.888 #1 derajat bumi = 111,32 km

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

#Landing
def land(vehicle:object):
    print('Landing...')
    change_mode(vehicle, 'LAND')
    print('Landed')

def maju(vehicle:object, distance:float, speed:float):
    vehicle.groundspeed = speed
    start = LocationGlobalRelative(
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon,
        vehicle.location.global_relative_frame.alt
    )
    target = gerak(vehicle,distance)
    vehicle.simple_goto(target)
    
    while True: #Tunggu sampai target jarak
        current = vehicle.location.global_relative_frame #Lokasi sekarang
        tempuh = get_distance_metres(start, current) #yang sudah ditempuh

        print(f"Tempuh: {tempuh:.2f} m")

        if tempuh >= distance-0.5: #toleransi 0.5 m
            print("Target tercapai")
            break

        time.sleep(0.2)

def yaw(vehicle:object, degree:float):
    send_yaw(vehicle,abs(degree))
    # heading awal
    start_yaw = vehicle.heading
    # hitung target absolut
    target_yaw = (start_yaw + degree) % 360 #karena heading dalam rentang 0-360
    tolerance= 5 #toleransi 5 derajat, agar loop bisa selesai

    while True:
        current_yaw = vehicle.heading
        # hitung selisih sudut terpendek
        diff = abs((target_yaw - current_yaw + 180) % 360 - 180)
        if diff <= tolerance:
            break
        time.sleep(0.5)

    print(f'Yaw {degree} derajat')