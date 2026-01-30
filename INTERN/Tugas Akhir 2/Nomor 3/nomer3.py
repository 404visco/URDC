#Setup
import collections
import cv2
import numpy as np
import math

import time
from collections import abc
collections.MutableMapping = abc.MutableMapping

from dronekit import connect,VehicleMode, LocationGlobalRelative

#delay
def delay(time):
    hitung = time
    while hitung>0:
        print(hitung)
        hitung-=1
        time.sleep(1)
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

def get_distance_metres(loc1, loc2):
    dlat = loc2.lat - loc1.lat #jarak latitude
    dlon = loc2.lon - loc1.lon #jarak longitude

    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 113195,4 #(113195,4: keliling bumi/360)

def record_environment(duration=5, waypoint_index=0):
    """
    Menyalakan Webcam Laptop sebagai simulasi kamera drone.
    """
    
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        return

    start_time = time.time()
    while time.time() - start_time < duration:
        ret, frame = cap.read()
        if ret:
            # Overlay Teks Status
            cv2.putText(frame, f"REC | WP: {waypoint_index}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow("Drone Camera Feed", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    cap.release()
    cv2.destroyAllWindows()
