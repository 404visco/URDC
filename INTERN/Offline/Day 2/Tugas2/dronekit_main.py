from dronekit_control import *
import global_variable as gv
from yolo import get_finger_count
import threading
import time

vehicle = connect_vehicle()

def camera_loop():
    while gv.running:
        fc = get_finger_count()
        if fc is not None:
            gv.finger_count = fc
        time.sleep(0.03)  # ~30 FPS

def is_takeoff(vehicle, min_alt= 5): #Untuk memisah kondisi
    alt = vehicle.location.global_relative_frame.alt
    return vehicle.armed and alt is not None and alt> min_alt
def is_landed(vehicle, alt_thresh=0.2):
    alt = vehicle.location.global_relative_frame.alt
    return (not vehicle.armed) and alt is not None and alt < alt_thresh


def hand_control():
    change_mode(vehicle, "GUIDED")
    while gv.running:
        if is_landed(vehicle):
            change_mode(vehicle, "GUIDED")
        if not is_takeoff(vehicle):
            f0(vehicle)
        else:
            vx, vy, vz = f1(vehicle)
            if vx or vy or vz:
                send_ned_velocity(vehicle, vx, vy, vz)
        time.sleep(0.1)


threading.Thread(target=camera_loop, daemon=True).start()
hand_control()

#Exit n reset
gv.current_velocity = (0, 0, 0)
send_ned_velocity(vehicle, 0, 0, 0)

vehicle.close()
cv2.destroyAllWindows()