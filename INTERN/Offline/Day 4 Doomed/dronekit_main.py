from dronekit import connect
import dronekit_control as dc
import yolo
import time
from global_variable import *

vehicle = connect('127.0.0.1:14550', wait_ready=True)

dc.arm_and_takeoff(vehicle, TARGET_ALT)

stable_count = 0
object_grabbed = False

while True:
    detected, cx, cy, frame = yolo.detect_object()

    vz = dc.altitude_hold(vehicle)

    # ===== fake lidar input =====
    lidar = {"right": 1.25, "left": 1.25}
    vy_wall = dc.wall_centering_controller(lidar)

    if detected and not object_grabbed:
        error_x = cx - FRAME_CX
        error_y = cy - FRAME_CY

        vx, vy_vision = dc.visual_controller(error_x, error_y)
        vy = vy_vision + vy_wall

        if abs(error_x) < VISION_DEADZONE_PX and abs(error_y) < VISION_DEADZONE_PX:
            stable_count += 1
        else:
            stable_count = 0

        if stable_count >= STABLE_REQUIRED:
            dc.send_ned_velocity(vehicle, 0, 0, 0)
            time.sleep(0.5)

            # turun ambil objek
            dc.send_ned_velocity(vehicle, 0, 0, 0.4)
            time.sleep(1)

            dc.grab_object(vehicle)
            object_grabbed = True

        else:
            dc.send_ned_velocity(vehicle, vx, vy, vz)

    elif object_grabbed:
        # naik kembali
        dc.send_ned_velocity(vehicle, 0, 0, -0.4)
        time.sleep(1)

        dc.release_object(vehicle)
        break

    else:
        dc.send_ned_velocity(vehicle, 0, vy_wall, vz)

vehicle.close()
