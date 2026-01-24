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
    return True

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
#Landing
def land(vehicle):
    print('Landing...')
    change_mode(vehicle,'LAND')
    print('Landed')

def send_ned_velocity(vehicle, vx,vy,vz):
    to_send = vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE,
        0,0,0,
        vx,vy,vz,
        0,0,0,
        0,0
        )
    vehicle.send_mavlink(to_send)

def send_yaw(vehicle,
             target_degree, #Target derajat
             v_yaw = 30, #kecepatan yaw derajat/detik
             arah = 1, # 1 = searah jarum jam, -1 = kebalikan jarum jam
             relative = True #True = relative dari arah sekarang, False = relatif kompas
             ):
    to_send = vehicle.message_factory.command_log_encode(
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
        hitung+=0.1
        sleep(0.1) #perintah maju harus dikirim berulang

def yaw(vehicle, degree):
    send_yaw(vehicle,degree)
    print('Yaw 90 derajat',)
    delay(3)