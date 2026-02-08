from dronekit_control import *
from yolo import YoloDetector
vehicle = connect_vehicle()
yolo = YoloDetector(
    model_path="model.pt",
    cam_index=0,
    conf=0.6
)
yolo.start()
start_lidar_listener(vehicle)
start_time = time.time()
while True:

    if gv.mission_state == gv.MISSION_TAKEOFF:
        takeoff(vehicle, gv.alt_target)
        gv.mission_state = gv.MISSION_CORRIDOR

    elif gv.mission_state == gv.MISSION_CORRIDOR:
        terbang(vehicle)
        if gv.object_detected and abs(gv.object_x_error) < 0.08 and abs(gv.object_y_error) < 0.08:
            gv.mission_state = gv.MISSION_DESCEND
        elif time.time() - start_time > 30:
            land()

    elif gv.mission_state == gv.MISSION_DESCEND:
        send_ned_velocity(vehicle, 0,0,0)
        time.sleep(2)
        turun(vehicle, 0.3)
        alt = vehicle.location.global_relative_frame.alt
        if alt <= 0.10:
            gv.mission_state = gv.MISSION_GRAB

    elif gv.mission_state == gv.MISSION_GRAB:
        set_servo(vehicle, 9, gv.capit)
        time.sleep(2)
        gv.mission_state = gv.MISSION_ASCEND

    elif gv.mission_state == gv.MISSION_ASCEND:
        naik(vehicle,0.4)
        alt = vehicle.location.global_relative_frame.alt
        if alt>= gv.alt_target*0.95:
            gv.mission_state = gv.MISSION_RELEASE

    elif gv.mission_state == gv.MISSION_RELEASE:
        time.sleep(2)
        set_servo(vehicle, 9, gv.lepas)
        break

land(vehicle)