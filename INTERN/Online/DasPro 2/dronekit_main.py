from dronekit_control import *

def using_keyboard():
    vehicle = connect_vehicle()
    
    change_mode(vehicle, "GUIDED")
    arm(vehicle)
    
    takeoff(vehicle, 1.25)
    delay(5)
    
    keyboard_teleop(vehicle)
    
    change_mode(vehicle, "LAND")
    delay(4)
    print("Program selesai")
    
def waypoint():
    vehicle = connect_vehicle()
    
    change_mode(vehicle, "GUIDED")
    arm(vehicle)
    
    takeoff(vehicle, 2.5)
    delay(5)
    
    send_global_position(vehicle, 1)
    delay(7)
    
    send_global_position(vehicle, 2)
    delay(7)
    
    send_global_position(vehicle, 3)
    delay(7)

    send_global_position(vehicle, 4)
    delay(7)

    send_global_position(vehicle, 5)
    delay(7)

    send_global_position(vehicle, 6)
    delay(7)

    send_global_position(vehicle, 7)
    delay(7)

    send_global_position(vehicle, 8)
    delay(7)

    send_global_position(vehicle, 9)
    delay(7)

    send_global_position(vehicle, 10)
    delay(7)
    
    send_global_position(vehicle, 1)
    delay(7)
    
    change_mode(vehicle, "LAND")

    

if __name__ == "__main__":
    using_keyboard()
    # waypoint()