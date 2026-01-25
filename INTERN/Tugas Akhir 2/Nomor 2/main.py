#Setup
import collections
import cv2
import numpy as np
import math

from time import sleep
from collections import abc
collections.MutableMapping = abc.MutableMapping

from dronekit import connect,VehicleMode, LocationGlobalRelative
from pymavlink import mavutil




#TELEMETRY
#----------------------------------------------------------------------------------------
import tkinter as tk
import threading
import queue
import time

telemetry_queue = queue.Queue()

def log(msg):
    timestamp = time.strftime("%H:%M:%S")
    telemetry_queue.put(f"[{timestamp}] {msg}")

def telemetry_window():
    root = tk.Tk()
    root.title("Drone Telemetry")
    root.geometry("420x280")
    root.attributes("-topmost", True)

    text = tk.Text(
        root,
        bg="black",
        fg="lime",
        font=("Consolas", 10),
        wrap="word"
    )
    text.pack(expand=True, fill="both")

    def update():
        while not telemetry_queue.empty():
            line = telemetry_queue.get()
            text.insert("end", line + "\n")
            text.see("end")
        root.after(100, update)

    update()
    root.mainloop()
#---------------------------------------------------------------------------------------------




#delay
def delay(time):
    hitung = time
    while hitung>0:
        print(hitung)
        hitung-=1
        sleep(1)
    print('lesgo!')

#Sambung drone
def connect_vehicle():
    vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
    return vehicle

#Change Mode
def change_mode(vehicle, flight_mode):
    vehicle.mode= VehicleMode(flight_mode) #Atur Mode
    vehicle.wait_for_mode(flight_mode) #Nunggu mode berubah
    if vehicle.mode.name == flight_mode:
        print(f"Masuk mode {flight_mode}🫡")
    else:
        print("Gagal ganti mode😥")

#Arm
def arm(vehicle):
    change_mode(vehicle, "GUIDED")
    print('Waiting for Arming...')
    vehicle.arm(wait=True) #Arming drone
    while not vehicle.armed:
        delay(1)
    print('Armed✅')

#Takeoff
def take0ff(vehicle,altitude):
    change_mode(vehicle,'GUIDED') #ubah mode dulu
    arm(vehicle) #arming
    print('Drone lagi takeoff')
    vehicle.simple_takeoff(altitude) #takeoff ke altitude
    while True:
        alt= vehicle.location.global_relative_frame.alt
        print(f'ALtitude: {alt}')
        if alt<altitude:
            print('.', end='')
        else:
            print('\nTakeoff done')
            break
def hover(lama):
    print('Hover:')
    hitung=0
    while True:
        if hitung<lama:
            print(hitung+1)
            hitung+=1
            sleep(1)
        else:
            print('Hover selesai')
            break
#Landing
def land(vehicle):
    print('Landing...')
    change_mode(vehicle,'LAND')
    print('Landed')

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

def send_yaw(vehicle,
             target_degree, #Target derajat
             v_yaw = 30, #kecepatan yaw derajat/detik
             arah = 1, # 1 = searah jarum jam, -1 = kebalikan jarum jam
             relative = True #True = relative dari arah sekarang, False = relatif kompas
             ):
    to_send = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        target_degree,
        v_yaw,
        arah,
        1 if relative else 0,
        0, 0, 0
    )
    vehicle.send_mavlink(to_send)

def realkoor_plus_10m(real_koor, NorthSouth, EastWest): #membuat koor m ke radian
    r_earth= 6378137 #radius bumi
    lat10m= NorthSouth/r_earth #radian lat= jarak/radius
    lon10m= EastWest/(r_earth*math.cos(math.radians(real_koor.lat))) # dikali cos(latitude) karena semakin dekat kutub, longitude semakin kecil

    newlat= real_koor.lat + math.degrees(lat10m) #koor asli ditambah koor plus 10m
    newlon= real_koor.lon + math.degrees(lon10m) 
    
    return LocationGlobalRelative(newlat,newlon,real_koor.alt) #koor baru

def gerak(vehicle,jarak):
    current_loc= vehicle.location.global_relative_frame
    arah_drjt =  vehicle.heading
    arah_rad = math.radians(arah_drjt)

    gerak_NorthSouth= jarak * math.cos(arah_rad)
    gerak_EastWest= jarak * math.sin(arah_rad)

    return realkoor_plus_10m(current_loc, gerak_NorthSouth, gerak_EastWest)

def get_distance_metres(loc1, loc2):
    dlat = loc2.lat - loc1.lat #jarak latitude
    dlon = loc2.lon - loc1.lon #jarak longitude

    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 111111 #Pythagoras+ubah derajat ke m

def tunggu_sampe(vehicle,target, toleransi = 0.5):
    while True:
        loc1= vehicle.location.global_relative_frame #lokasi sebelum gerak
        jarak= get_distance_metres(loc1,target)
        print(jarak)

        if jarak<=toleransi:
            print(f'Target {jarak}m tercapai')
            break
        sleep(0.5)


def maju(vehicle, kecepatan, lama):
    hitung = 0
    print(f'Drone Maju selama {lama} detik dengan kecepatan {kecepatan} m/s')
    while hitung<lama:
        vx = vy = vz = 0
        vx += kecepatan
        send_ned_velocity(vehicle, vx, vy , vz)
        hitung+=0.1
        sleep(0.1) #perintah maju harus dikirim berulang
        
def kanan(vehicle, kecepatan, lama):
    hitung = 0
    print(f'Drone ke Kanan selama {lama} detik dengan kecepatan {kecepatan} m/s')
    while hitung<lama:
        vx = vy = vz = 0
        vy += kecepatan
        send_ned_velocity(vehicle, vx, vy, vz)
        if hitung == 1 or hitung ==2  or hitung ==3 or hitung ==4:
            print(hitung)
        hitung+=0.1
        sleep(0.1) #perintah maju harus dikirim berulang

def yaw(vehicle, degree):
    send_yaw(vehicle,degree)
    print('Yaw 90 derajat',)
    delay(4)

def maju_loc(vehicle,jarak):
    target = gerak(vehicle,jarak)
    vehicle.simple_goto(target)
    tunggu_sampe(vehicle,target,jarak)