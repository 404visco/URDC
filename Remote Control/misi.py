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
    vehicle = connect_vehicle()
    arm(vehicle)

    take0ff(vehicle, 1.5) #Takeoff 1.5 m
    hover(5) #Hover 5 detik stelah tsKEOFF
    maju(vehicle, 0.25, 4) #maju 4 detik, kecepatan 0.25 m/s
    kanan(vehicle, 0.25, 4) #ke kanan 4 detik, kecepatan 0.25 m/s
    yaw(vehicle, 90) #yaw 90 derajat
    land(vehicle) #Landing
    maju(vehicle,10) #Maju 10 m
    
if __name__ == "__main__":
    run_misi()