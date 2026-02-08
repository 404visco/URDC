dist_left = None
dist_right = None
wall_dist = 2.5
lebar_drone = 0.6
tinggi_drone = 0.3
alt_target= 1.2
speed_drone = 0.5
takeoff2object = 2.45

object_detected = False
object_x_error = 0.0  
object_y_error = 0.0

basket_detected = False
basket_x_error = 0.0  
basket_y_error = 0.0

capit = 1000
lepas = 2000

dist_front = None
front_target = 1.25      # jarak target ke tembok depan (meter)
front_tol = 0.15


MISSION_TAKEOFF = 0
MISSION_CORRIDOR = 1
MISSION_DETECT = 2
MISSION_DESCEND = 3
MISSION_GRAB = 4
MISSION_ASCEND = 5
MISSION_RELEASE = 6
MISSION_ANTER1 = 7
MISSION_APPROACH_FRONT = 8
MISSION_YAW = 9
MISSION_FORWARD_AFTER_YAW = 10
MISSION_OBJEK2 =11
MISSION_YAWMUTER =12

VISION_OBJECT = 1
VISION_BASKET = 2

vision_mode = VISION_OBJECT

mission_state = MISSION_TAKEOFF

yaw_margin = 0.3   # meter, beda minimal agar keputusan valid
