import global_variable as gv
from dronekit import VehicleMode,connect, LocationGlobalRelative
from pymavlink import mavutil
import collections
from collections import abc
collections.MutableMapping = abc.Mutablemapping
import time
import math

def connect_vehicle():
    vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
    return vehicle

def change_mode(vehicle:object, flight_mode:str):
    vehicle.mode = VehicleMode(flight_mode)
    vehicle.wait_for_mode(flight_mode)
    if not vehicle.mode.name == flight_mode:
        print('Failed to change mode')
    print(f'Flight mode changed to {flight_mode}')

def arm(vehicle):
    print("Arming..")
    vehicle.arm(wait= True)
    while not vehicle.armed:
        print('Wait...')
    print('Vehicle Armed')

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

def takeoff(vehicle:object, altitude:float):
    arm(vehicle)
    vehicle.simple_takeoff(altitude)
    while True: #Tunggu sampai drone sudah mencapai target altitude
        alt0 = vehicle.location.global_relative_frame.alt #pantau altitude
        if alt0 >= altitude*0.95: #toleransi agar tidak loop selamanya
            break
        time.sleep(1)

def maju(vehicle, speed):
    vx = vy = vz = 0 #awal semua 0
    vx= +speed
    send_ned_velocity(vehicle,vx,vy,vz)
def mundur(vehicle,speed):
    vx = vy = vz = 0 #awal semua 0
    vx= -speed
    send_ned_velocity(vehicle,vx,vy,vz)
def kanan(vehicle,speed):
    vx = vy = vz = 0 #awal semua 0
    vy= +speed
    send_ned_velocity(vehicle,vx,vy,vz)
def kiri(vehicle,speed):
    vx = vy = vz = 0 #awal semua 0
    vy= -speed
    send_ned_velocity(vehicle,vx,vy,vz)
def naik(vehicle,speed):
    vx = vy = vz = 0 #awal semua 0
    vz= -speed
    send_ned_velocity(vehicle,vx,vy,vz)
def turun(vehicle,speed):
    vx = vy = vz = 0 #awal semua 0
    vz= +speed
    send_ned_velocity(vehicle,vx,vy,vz)
def alt_hold(vehicle):
    alt = vehicle.location.global_relative_frame.alt
    error = gv.alt_target - alt
    vx=vy=vz=0
    if abs(error) < 0.5:
        send_ned_velocity(vehicle,vx,vy,vz)
    else:
        vz = 0.4 if error < 0 else -0.4
        send_ned_velocity(vehicle, vx,vy,vz)
def land(vehicle:object):
    print('Landing...')
    change_mode(vehicle, 'LAND')
    print('Landed')

def lidar_callback(self, name, msg):
    if msg.current_distance <= 0:
        return

    dist = msg.current_distance / 100.0  # cm → meter

    if msg.orientation == mavutil.mavlink.MAV_SENSOR_ROTATION_LEFT:
        gv.dist_left = dist

    elif msg.orientation == mavutil.mavlink.MAV_SENSOR_ROTATION_RIGHT:
        gv.dist_right = dist
    elif msg.orientation == mavutil.mavlink.MAV_SENSOR_ROTATION_FORWARD:
        gv.dist_front = dist



def start_lidar_listener(vehicle:object):
    vehicle.add_message_listener('DISTANCE_SENSOR', lidar_callback)

def terbang(vehicle):
    if gv.dist_left is None or gv.dist_right is None:
        return
    koreksi = gv.dist_left - gv.dist_right #buat tau drone perlu gerak ke mana
    vy = -0.4*koreksi
    vy = max(min(vy, 0.3), -0.3)
    alt = vehicle.location.global_relative_frame.alt
    alt_error = gv.alt_target - alt
    vz = max(min(0.4 * alt_error, 0.3), -0.3)
    if gv.object_detected:
        vx = -0.4 * gv.object_y_error
        vx = max(min(vx, 0.3), -0.3)
    else:
        vx = gv.speed_drone

    send_ned_velocity(vehicle, vx,vy,vz)

def set_servo(vehicle, channel, pwm):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm,
        0,0,0,0,0
    )
    vehicle.send_mavlink(msg)


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

def yaw(vehicle:object, degree:float):
    send_yaw(vehicle,abs(degree), 1 if degree>=0 else -1)
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


def approach_front_wall(vehicle):
    if gv.dist_front is None:
        send_ned_velocity(vehicle, 0,0,0)
        return False

    error = gv.dist_front - gv.front_target
    vx = 0.4 * error
    vx = max(min(vx, 0.3), -0.3)

    send_ned_velocity(vehicle, vx, 0, 0)

    return abs(error) < gv.front_tol

def decide_yaw_direction():
    if gv.dist_left is None or gv.dist_right is None:
        return None

    diff = gv.dist_right - gv.dist_left

    if diff > gv.yaw_margin:
        return "RIGHT"
    elif diff < -gv.yaw_margin:
        return "LEFT"
    else:
        return "CENTER"
