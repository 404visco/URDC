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

capit = 1000
lepas = 2000

MISSION_TAKEOFF = 0
MISSION_CORRIDOR = 1
MISSION_DETECT = 2
MISSION_DESCEND = 3
MISSION_GRAB = 4
MISSION_ASCEND = 5
MISSION_RELEASE = 6

mission_state = MISSION_TAKEOFF