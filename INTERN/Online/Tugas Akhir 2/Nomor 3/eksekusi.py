from nomer3 import *

def eksekusi():
    vehicle= connect_vehicle()

    change_mode(vehicle, 'GUIDED')
    arm(vehicle)

    takeoff(vehicle, 8)
    delay(15)

    #eksekusi
    send_global_position(vehicle, 1)
    record_environment()
    delay(3)
    send_global_position(vehicle, 2)
    record_environment()
    delay(3)
    send_global_position(vehicle, 3)
    record_environment()
    delay(3)
    send_global_position(vehicle, 4)
    record_environment()
    delay(3)
    send_global_position(vehicle, 5)
    record_environment()
    delay(3)

    #balik
    change_mode(vehicle, "RTL")
    delay(15)
    change_mode(vehicle, "LAND")

eksekusi()