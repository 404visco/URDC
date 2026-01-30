from nomer3 import *
vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)

def get_distance_metres(loc1, loc2):
    dlat = loc2.lat - loc1.lat #jarak latitude
    dlon = loc2.lon - loc1.lon #jarak longitude

    return math.sqrt((dlat*dlat) + (dlon*dlon)) * 113195,4 #(113195,4: keliling bumi/360)