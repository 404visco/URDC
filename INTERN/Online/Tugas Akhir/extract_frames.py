import cv2 #Computer Vision
import os #Untuk ambil file

#Extarct frames dari video
def extract_frames(video_file, interval):
    #Open video file
    cap = cv2.VideoCapture(video_file)

    if not cap.isOpened(): #Kalo videonya gabisa dibuka
        print('Error: Unable to open  video file %s' % video_file)
        return
    
    #Kasih Tempat buat nyimpen frame
    video_name = os.path.splitext(os.path.basename(video_file))[0]
    output_dir = video_name

    if not os.path.exists(output_dir): #Kalo directori/tempat belum ada
        os.makedirs(output_dir) #buat directori

    frame_count= 0 
    extracted_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_count < interval:
            frame_filename = '%s/frame_%04d.jpg' % (output_dir, extracted_count)
            cv2.imwrite(frame_filename, frame)
            extracted_count += 1

        frame_count += 1

    cap.release()
    print(f"Selesai. {extracted_count} frame diekstrak.")

video_path= 'C:/Users/visco/JENTAYU/INTERN/Online/Tugas Akhir/rubik.mp4'

extract_frames(video_path, 1500)