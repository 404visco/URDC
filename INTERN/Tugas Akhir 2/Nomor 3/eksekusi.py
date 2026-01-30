from nomer3 import *

def eksekusi():
    vehicle= connect_vehicle()

    change_mode(vehicle, 'GUIDED')
    arm(vehicle)

    takeoff(vehicle, 5)
    delay(3)

    #eksekusi
    send_global_position(vehicle, 1)
    record_environment()
    send_global_position(vehicle, 2)
    record_environment()
    send_global_position(vehicle, 3)
    record_environment()
    send_global_position(vehicle, 4)
    record_environment()
    send_global_position(vehicle, 5)
    record_environment()

    #balik
    vehicle.mode = VehicleMode("RTL")

    vehicle.close()

eksekusi()