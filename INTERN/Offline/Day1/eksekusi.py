from control import *
#Sesuai dengan rencana yang sudah dibuat

# •	Takeoff dari Start
# •	Maju ke Wp 1, kecepatan 0.2 m/s
# •	Yaw -90 derajat
# •	Maju ke Wp 2, kecepatan 0.6 m/s
# •	Yaw 90 derajat
# •	Maju ke Wp 3, kecepatan 0.4 m/s
# •	Maju ke Wp 4, kecepatan 0.4 m/s
# •	Maju ke Wp 5, keccepatan 0.4 m/s
# •	Maju ke Wp 6, kecepatan 0.4 m/s
# •	Yaw 90 derajat
# •	Maju ke Wp 7, kecepatan 0.4 m/s
# •	Maju ke Wp 8, kecepatan 0.4 m/s
# •	Yaw -90 derajat
# •	Maju ke Wp 9, kecepatan 0.4 m/s
# •	Maju ke Wp 10, kecepatan 0.4 m/s
# •	Yaw 90 derajat
# •	Maju menuju Finish, kecepatan 0.6 m/s

#JARAK

# 	Start  Wp1:  20 cm = 0,2 m
# 	Wp1  Wp2: 180 cm = 1,8 m
# 	Wp2  Wp3: 80 cm = 0,8 m
# 	Wp3  Wp4: 80 cm = 0,8 m
# 	Wp4  Wp5: 60 cm = 0,6 m
# 	Wp5  Wp6: 60 cm = 0,6 m
# 	Wp6  Wp7: 80 cm = 0,8 m
# 	Wp7  Wp8: 60 cm = 0,6 m
# 	Wp8  Wp9: 40 cm = 0,4 m
# 	Wp9  Wp10: 40 cm = 0,4 m
# 	Wp10  Finish : 100 cm = 1 m

def run():
    vehicle = connect_vehicle()
    change_mode(vehicle, 'GUIDED')
    arm(vehicle)

    takeoff(vehicle, 2)
    #START --> Wp1
    maju(vehicle, 0.2, 0.2)
    yaw(vehicle,-90)
    #Wp1 --> Wp2
    maju(vehicle,1.8, 0.6)
    yaw(vehicle,90)
    #Wp2 --> Wp3
    maju(vehicle,0.8, 0.4)
    #Wp3 --> Wp4
    maju(vehicle, 0.8, 0.4)
    #Wp4 --> Wp5
    maju(vehicle, 0.6, 0.4)
    #Wp5 --> Wp6
    maju(vehicle, 0.6, 0.4)
    yaw(vehicle, 90)
    #Wp6 --> Wp7
    maju(vehicle,0.8, 0.4)
    #Wp7 --> Wp8
    maju(vehicle,0.8, 0.4)
    yaw(vehicle, -90)
    #Wp8 --> Wp9
    maju(vehicle,0.4, 0.4)
    #Wp9 --> Wp10
    maju(vehicle,0.4, 0.4)
    yaw(vehicle, 90)
    #Wp10 --> Finish
    maju (vehicle, 1.0, 0.6)
    land(vehicle)

run()