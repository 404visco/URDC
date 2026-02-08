from dronekit_control import *
from yolo import YoloDetector,ObjectDetector
vehicle = connect_vehicle()
yolo = YoloDetector(
    model_path="model.pt",
    cam_index=0,
    conf=0.6
)
basket = ObjectDetector(
    model_path="model.pt",
    cam_index=1,
    conf=0.6
)
yolo.start()
basket.start()
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
        gv.vision_mode = gv.VISION_BASKET
        gv.mission_state = gv.MISSION_ASCEND

    elif gv.mission_state == gv.MISSION_ASCEND:
        naik(vehicle,0.4)
        alt = vehicle.location.global_relative_frame.alt
        if alt >= gv.alt_target*0.95:
            gv.mission_state = gv.MISSION_APPROACH_FRONT

    elif gv.mission_state == gv.MISSION_APPROACH_FRONT:
        done = approach_front_wall(vehicle)
        if done:
            send_ned_velocity(vehicle,0,0,0)
            gv.mission_state = gv.MISSION_YAW

    elif gv.mission_state == gv.MISSION_YAW:
            arah = decide_yaw_direction()

            if arah == "RIGHT":
                yaw(vehicle, 90)
                gv.mission_state = gv.MISSION_FORWARD_AFTER_YAW

            elif arah == "LEFT":
                yaw(vehicle, -90)
                gv.mission_state = gv.MISSION_FORWARD_AFTER_YAW

            else:
                send_ned_velocity(vehicle, 0, 0, 0)
                # tunggu sampai sensor jelas

    elif gv.mission_state == gv.MISSION_FORWARD_AFTER_YAW:
        terbang(vehicle)
        if gv.object_detected and abs(gv.object_x_error) < 0.08 and abs(gv.object_y_error) < 0.08:
            gv.mission_state = gv.MISSION_RELEASE
        elif time.time() - start_time > 30:
            land()

    elif gv.mission_state == gv.MISSION_RELEASE:
        time.sleep(2)
        set_servo(vehicle, 9, gv.lepas)
        gv.vision_mode = gv.VISION_OBJECT
        gv.mission_state= gv.OBJEK2

    elif gv.mission_state == gv.MISSION_OBJEK2:
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
        gv.vision_mode = gv.VISION_BASKET
        gv.mission_state = gv.MISSION_ASCEND

    elif gv.mission_state == gv.MISSION_ASCEND:
        naik(vehicle,0.4)
        alt = vehicle.location.global_relative_frame.alt
        if alt >= gv.alt_target*0.95:
            gv.mission_state = gv.MISSION_YAWMUTER

    elif gv.mission_state == gv.MISSION_YAWMUTER:
        yaw(vehicle, 90)   # yaw kanan 90°
        gv.mission_state = gv.MISSION_FORWARD_AFTER_YAW

    elif gv.mission_state == gv.MISSION_FORWARD_AFTER_YAW:
        terbang(vehicle)
        if gv.object_detected and abs(gv.object_x_error) < 0.08 and abs(gv.object_y_error) < 0.08:
            gv.mission_state = gv.MISSION_RELEASE
        elif time.time() - start_time > 30:
            land()

    elif gv.mission_state == gv.MISSION_RELEASE:
        time.sleep(2)
        set_servo(vehicle, 9, gv.lepas)
        break
    
land(vehicle)