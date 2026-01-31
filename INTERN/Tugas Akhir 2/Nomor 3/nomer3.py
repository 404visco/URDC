#Setup
import collections
import cv2
import numpy as np
import math
from pymavlink import mavutil

from time import sleep
import time
from collections import abc
collections.MutableMapping = abc.MutableMapping

from dronekit import connect,VehicleMode, LocationGlobalRelative

import global_variable

#delay
def delay(time):
    hitung = time
    while hitung>0:
        print(hitung)
        hitung-=1
        sleep(1)
    print('lesgo!')

def connect_vehicle():
    vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
    return vehicle

# Ubah Mode Terbang
def change_mode(vehicle: object, flight_mode: str) -> None:
    vehicle.mode = VehicleMode(flight_mode)
    vehicle.wait_for_mode(flight_mode)
    if vehicle.mode.name == flight_mode:
        print(f"Mode changed to {flight_mode}")
    else:
        print("Failed to change mode")

# Arming
def arm(vehicle: object):
    print("Arming motors")
    vehicle.arm(wait=True)
    while not vehicle.armed:
        print("Still disarmed...")
        delay(1)
        
    print("\nArmed")

def takeoff(vehicle: object, target_altitude: float):
    change_mode(vehicle, "GUIDED")
    arm(vehicle)
    
    while not vehicle.armed:
        print("Waiting for arming...")
        delay(1)
    
    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)

def send_global_position(vehicle: object, code: int):
    latitude = global_variable.waypoint[code]["latitude"]
    longitude = global_variable.waypoint[code]["longitude"]
    altitude = global_variable.waypoint[code]["altitude"]
    
    to_send = vehicle.message_factory.set_position_target_global_int_encode(
        10, 0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        0b110111111000,
        int(latitude * 10**7),
        int(longitude * 10**7),
        altitude,
        0, 0, 0,
        0, 0, 0,
        0, 0)
    
    vehicle.send_mavlink(to_send)

def record_environment(duration=5, waypoint_index=0):
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        return
    start_time = time.time()
    while time.time() - start_time < duration:
        ret, frame = cap.read()
        if ret:
            # Window
            cv2.putText(frame, f"REC | WP: {waypoint_index}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Drone Camera Feed", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap.release()
    cv2.destroyAllWindows()