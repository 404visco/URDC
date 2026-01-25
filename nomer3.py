from dronekit import connect, VehicleMode, LocationGlobalRelative
import cv2
import time

# 1. Koneksi ke Simulator (SITL)
print("Menghubungkan ke drone...")
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def rekam_wilayah(nama_titik):
    # Membuka kamera (0 adalah webcam bawaan)
    cap = cv2.VideoCapture(0)
    
    ret, frame = cap.read()
    if ret:
        # Memberi teks pada gambar agar terlihat profesional
        cv2.putText(frame, f"Titik: {nama_titik}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Simpan gambar sebagai bukti perekaman
        cv2.imwrite(f"hasil_peta_{nama_titik}.jpg", frame)
        print(f"Foto wilayah {nama_titik} berhasil diambil.")
    
    cap.release()

def fly_to_point(lat, lon, alt, name):
    print(f"Menuju ke {name}...")
    target_location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(target_location)
    
    # Logika berhenti untuk merekam:
    # Kita cek apakah drone sudah sampai (jarak < 1 meter)
    while True:
        # (Logika perhitungan jarak di sini)
        # Jika sudah sampai:
        time.sleep(5) # Berhenti sejenak
        rekam_wilayah(name)
        break