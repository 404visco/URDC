from main import *
'''
MISI
 Takeoff hingga 1.5 m dari atas tanah
 Hover selama 5 detik setelah takeoff
 Maju dengan kecepatan 0.25 m/s selama 4 detik
 Ke kanan dengan kecepatan 0.25 m/s selama 4 detik
 Yaw 90o
 Maju (dengan fungsi pindah lokasi) sejauh 10 m
 Landing
'''

def run_misi():
    vehicle = connect_vehicle
    arm(vehicle)

    take0ff(vehicle, 1.5) #Takeoff 1.5 m
    