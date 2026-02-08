import global_variable as gv
from dronekit import VehicleMode,connect
from pymavlink import mavutil
import collections
from collections import abc
collections.MutableMapping = abc.Mutablemapping
import time

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

def maju(speed):
    vx = vy = vz = 0 #awal semua 0
    vx= +speed
    send_ned_velocity(vx,vy,vz)
def mundur(speed):
    vx = vy = vz = 0 #awal semua 0
    vx= -speed
    send_ned_velocity(vx,vy,vz)
def kanan(speed):
    vx = vy = vz = 0 #awal semua 0
    vy= +speed
    send_ned_velocity(vx,vy,vz)
def kiri(speed):
    vx = vy = vz = 0 #awal semua 0
    vy= -speed
    send_ned_velocity(vx,vy,vz)
def naik(speed):
    vx = vy = vz = 0 #awal semua 0
    vz= -speed
    send_ned_velocity(vx,vy,vz)
def turun(speed):
    vx = vy = vz = 0 #awal semua 0
    vz= +speed
    send_ned_velocity(vx,vy,vz)
def alt_hold(vehicle):
    alt = vehicle.location.global_relative_frame.alt
    error = gv.alt_target - alt
    vx=vy=vz=0
    if abs(error) < 0.5:
        send_ned_velocity(vehicle,vx,vy,vz)
    else:
        vz = 0.4 if error < 0 else -0.4
        send_ned_velocity(vehicle, vx,vy,vz)

def lidar_callback(self, name, msg):
    if msg.current_distance <= 0:
        return

    dist = msg.current_distance / 100.0  # cm → meter

    if msg.orientation == mavutil.mavlink.MAV_SENSOR_ROTATION_LEFT:
        gv.dist_left = dist

    elif msg.orientation == mavutil.mavlink.MAV_SENSOR_ROTATION_RIGHT:
        gv.dist_right = dist


def start_lidar_listener(vehicle:object):
    vehicle.add_message_listener('DISTANCE_SENSOR', lidar_callback)

def terbang(vehicle):
    if gv.dist_left is None or gv.dist_right is None:
        return
    koreksi = gv.dist_left - gv.dist_right #buat tau drone perlu gerak ke mana
    vy = -0.4*koreksi
    vy = max(min(vy, 0.2), -0.2)
    alt = vehicle.location.global_relative_frame.alt
    alt_error = gv.alt_target - alt
    vz = max(min(0.4 * alt_error, 0.3), -0.3)
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