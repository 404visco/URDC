from main import *
def delay(time):
    hitung = time
    while hitung>0:
        print(hitung)
        hitung-=1
        sleep(1)
delay(4)