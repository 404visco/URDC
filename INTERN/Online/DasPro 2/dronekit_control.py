import collections
import cv2
import numpy as np
import global_variable
from time import sleep
from collections import abc
collections.MutableMapping = abc.MutableMapping
from dronekit import connect, VehicleMode
from pymavlink import mavutil

def delay(t: int):
    for i in range(1, t+1):
        print(i, end=" ")
        sleep(1)

# Hubungkan wahana
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

# Disarming  
def disarm(vehicle: object):
    print("Disarming motors")
    vehicle.disarm(wait=True)
    print("Vehicle Disarmed")

# Takeoff
def takeoff(vehicle: object, target_altitude: float):
    change_mode(vehicle, "GUIDED")
    arm(vehicle)
    
    while not vehicle.armed:
        print("Waiting for arming...")
        delay(1)
    
    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)

# Landing
def land(vehicle: object):
    print("Landing...")
    change_mode(vehicle, "LAND")
    print("Vehicle landed")
    
# Berdasarkan Kecepatan
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
# Berdasarkan Sikap
def send_attitude(vehicle, roll_rate, pitch_rate, yaw_rate, thrust):
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0,
        0b10000000,   # ifull rate
        [1,0,0,0], #dummy quaternion
        roll_rate, pitch_rate, yaw_rate,
        thrust
    )
    vehicle.send_mavlink(msg)

#Berdasarkan Lokasi
def send_local_position(vehicle, x, y, z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111111000,  # hanya posisi 
        x, y, z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)


def send_global_position(vehicle: object, code: int):
    latitude = global_variable.waypoint_item[code]["latitude"]
    longitude = global_variable.waypoint_item[code]["longitude"]
    altitude = global_variable.waypoint_item[code]["altitude"]
    
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

#velocitykey
def vkey(key, speed_xy=1, speed_z=0.5):
        vx = vy = vz = 0 #awal semua 0
        #Maju
        if key in (ord('w'), ord('W')):
            vx = +speed_xy
            print(f'forward {vx} m/s')
        #Mundur
        elif key in (ord('s'), ord('S')):
            vx = -speed_xy
            print(f'backward {vx} m/s')
        #Kanan
        if key in (ord('d'), ord('D')):
            vy = +speed_xy
            print(f'to right {vy} m/s')
        #Kiri
        elif key in (ord('a'), ord('A')):
            vy = -speed_xy
            print(f'to_left {vy} m/s')
        #Atas
        if key in (ord('P'), ord('p')): #Panah Atas
            vz = -speed_z
            print(f'up {vz} m/s')
        #Bawah
        elif key in (ord('L'), ord('l')): #Panah Bawah
            vz = +speed_z
            print(f'down {vz} m/s')
        return vx, vy ,vz
#attitudekey
def akey(key,
         rate_step=0.1,      #rad/s
         yaw_rate_step=0.3, #rad/s  
         thrust_step=0.05,
         base_thrust=0.5):

    # default: tidak ada gerakan
    roll_rate= pitch_rate = yaw_rate = 0
    thrust = base_thrust

    # PITCH
    if key in (ord('w'), ord('W')):
        pitch_rate = -rate_step
        print("ATT: pitch forward")
    elif key in (ord('s'), ord('S')):
        pitch_rate = rate_step
        print("ATT: pitch backward")

    # ROLL
    if key in (ord('d'), ord('D')):
        roll_rate = rate_step
        print("ATT: roll right")
    elif key in (ord('a'), ord('A')):
        roll_rate = -rate_step
        print("ATT: roll left")

    # YAW (rate)
    if key in (ord('e'), ord('E')):
        yaw_rate = +yaw_rate_step
        print("ATT: yaw right")
    elif key in (ord('q'), ord('Q')):
        yaw_rate = -yaw_rate_step
        print("ATT: yaw left")

    # THROTTLE
    if key in (ord('p'), ord('P')):
        thrust = min(base_thrust + thrust_step, 1.0)
        print("ATT: throttle up")
    elif key in (ord('l'), ord('L')):
        thrust = max(base_thrust - thrust_step, 0.0)
        print("ATT: throttle down")

    return roll_rate, pitch_rate, yaw_rate, thrust

#positionkey
def pkey(key, step_xy=1, step_z=0.5):
    x = y = z = 0.0

    # MAJU / MUNDUR
    if key in (ord('w'), ord('W')):
        x = step_xy
        print("POS: forward")
    elif key in (ord('s'), ord('S')):
        x = -step_xy
        print("POS: backward")

    # KIRI / KANAN
    if key in (ord('d'), ord('D')):
        y = step_xy
        print("POS: right")
    elif key in (ord('a'), ord('A')):
        y = -step_xy
        print("POS: left")

    # ATAS / BAWAH
    if key in (ord('p'), ord('P')):
        z = -step_z
        print("POS: up")
    elif key in (ord('l'), ord('L')):
        z = step_z
        print("POS: down")

    return x, y, z

#mode
def flightmode(key,vehicle):
        if key == ord('1'):
            print("STABILIZE")
            return change_mode(vehicle, "STABILIZE")
        elif key == ord('2'):
            print("LOITER")
            return change_mode(vehicle, "LOITER")
        elif key == ord('3'):
            print("ALTHOLD")
            return change_mode(vehicle, "ALT_HOLD")
        elif key == ord('4'):
            print("AUTO")
            return change_mode(vehicle, "AUTO")
        elif key == ord('5'):
            print("LAND")
            return change_mode(vehicle, "LAND")
        elif key == ord('0'):
            print("GUIDED")
            return change_mode(vehicle, "GUIDED")

def controlmode_key(key):
    if key in (ord('z'), ord('Z')):
        global_variable.control_mode = "VELOCITY"
        print("MODE VELOCITY")
        return "VELOCITY"
    elif key in (ord('x'), ord('X')):
        global_variable.control_mode = "ATTITUDE"
        print("MODE ATTITUDE")
        return "ATTITUDE"
    elif key in (ord('c'), ord('C')):
        global_variable.control_mode = "POSITION"
        print("MODE POSITION")
        return "POSITION"

def keyboard_teleop(vehicle, dt=0.1):
    cv2.namedWindow("teleop")
    img = np.zeros((120, 420, 3), dtype=np.uint8)

    print("Kontrol Drone:")
    print("W/A/S/D = Maju/Kiri/Mundur/Kanan")
    print("E/Q = Yaw Right/Left | P/L = Naik/Turun")
    print("Z = MODE VELOCITY, X = MODE ATTITUDE, C = MODE POSITION")
    print("SPACE = Stop | ESC = Keluar")

    # Set mode awal
    global_variable.control_mode = "VELOCITY"

    while True:
        cv2.imshow("teleop", img)
        key = cv2.waitKey(int(dt * 1000))

        # Keluar
        if key == 27:
            print("Keluar Teleop")
            break

        # Stop / hover
        if key == 32:  # Spasi
            send_ned_velocity(vehicle, 0, 0, 0)
            send_attitude(vehicle, 0, 0, 0, 0.5)
            continue

        # Ganti mode
        controlmode_key(key)
        flightmode(key,vehicle)
        # Pilih mode gerakan
        if global_variable.control_mode == "VELOCITY":
            vx, vy, vz = vkey(key)
            if vx or vy or vz:
                send_ned_velocity(vehicle, vx, vy, vz)

        elif global_variable.control_mode == "ATTITUDE":
            roll, pitch, yaw_rate, thrust = akey(key)
            if roll or pitch or yaw_rate or thrust != 0.5:
                send_attitude(vehicle, roll, pitch, yaw_rate, thrust)

        elif global_variable.control_mode == "POSITION":
            x, y, z = pkey(key)
            if x or y or z:
                send_local_position(vehicle, x, y, z)

    cv2.destroyWindow("teleop")



