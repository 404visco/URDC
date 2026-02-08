from dronekit_control import *
from yolo import YoloDetector
vehicle = connect_vehicle()
yolo = YoloDetector(
    model_path="model.pt",
    cam_index=0,
    conf=0.6
)
yolo.start()
takeoff(vehicle, gv.alt_target)
start_lidar_listener(vehicle)
while True:

    if gv.mission_state == gv.MISSION_TAKEOFF:
        takeoff(vehicle, gv.alt_target)
        gv.mission_state = gv.MISSION_CORRIDOR

    elif gv.mission_state == gv.MISSION_CORRIDOR:
        terbang(vehicle)
        if gv.object_detected:
            gv.mission_state = gv.MISSION_DESCEND

    elif gv.mission_state == gv.MISSION_DESCEND:
        turun(vehicle)
        alt = vehicle.location.global_relative_frame.alt
        if alt <= gv.tinggi_drone:
            gv.mission_state = gv.MISSION_GRAB

    elif gv.mission_state == gv.MISSION_GRAB:
        set_servo(vehicle, 9, gv.capit)
        gv.mission_state = gv.MISSION_ASCEND

    elif gv.mission_state == gv.MISSION_ASCEND:
        naik(vehicle)
        alt = vehicle.location.global_relative_frame.alt
        if alt>= gv.alt_target*0.95:
            gv.mission_state = gv.MISSION_RELEASE

    elif gv.mission_state == gv.MISSION_RELEASE:
        time.sleep(2)
        set_servo(vehicle, 9, gv.lepas)
        break
