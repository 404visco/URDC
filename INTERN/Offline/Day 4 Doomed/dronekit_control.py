from dronekit import VehicleMode
from pymavlink import mavutil
import time
from global_variable import *

# ===== state for low-pass filter =====
vx_prev = 0.0
vy_prev = 0.0


# ================= BASIC COMMAND =================

def arm_and_takeoff(vehicle, alt):
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(alt)

    while True:
        if vehicle.location.global_relative_frame.alt >= alt * 0.95:
            break
        time.sleep(0.5)


def send_ned_velocity(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)


# ================= CONTROLLERS =================

def altitude_hold(vehicle):
    alt = vehicle.location.global_relative_frame.alt
    error = TARGET_ALT - alt

    if abs(error) < ALT_TOL:
        return 0

    return -0.4 if error > 0 else 0.4


def apply_deadzone(err, zone):
    return 0 if abs(err) < zone else err


def visual_controller(error_x, error_y):
    global vx_prev, vy_prev

    error_x = apply_deadzone(error_x, VISION_DEADZONE_PX)
    error_y = apply_deadzone(error_y, VISION_DEADZONE_PX)

    vx_raw = KP_VISION * error_y
    vy_raw = KP_VISION * error_x

    vx = LOWPASS_ALPHA * vx_prev + (1 - LOWPASS_ALPHA) * vx_raw
    vy = LOWPASS_ALPHA * vy_prev + (1 - LOWPASS_ALPHA) * vy_raw

    vx = max(min(vx, MAX_VEL), -MAX_VEL)
    vy = max(min(vy, MAX_VEL), -MAX_VEL)

    vx_prev, vy_prev = vx, vy
    return vx, vy


def wall_centering_controller(dist):
    error = dist["right"] - dist["left"]

    if abs(error) < 0.05:
        return 0

    vy = KP_WALL * error
    return max(min(vy, 0.3), -0.3)


# ================= SERVO =================

def set_servo(vehicle, pwm):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        SERVO_CHANNEL,
        pwm,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)


def grab_object(vehicle):
    set_servo(vehicle, SERVO_GRAB)


def release_object(vehicle):
    set_servo(vehicle, SERVO_RELEASE)
