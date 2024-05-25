import cv2
import numpy as np
import time

# tạo đối tượng cascade phát hiện khuôn mặt 
pengklasifikasiWajah  = cv2.CascadeClassifier(r"C:\Users\hi\Downloads\Face-Detection-main\Face-Detection-main\haarcascade_frontalface_default.xml")

videoCam = cv2.VideoCapture(0)

if not videoCam.isOpened():
    print("Kamera tidak dapat diakses")
    exit()

tombolQditekan = False
while (tombolQditekan == False):
    ret, kerangka = videoCam.read()

    if ret == True:
        abuAbu = cv2.cvtColor(kerangka, cv2.COLOR_BGR2GRAY)  # chuyển sang grayscale
        dafWajah = pengklasifikasiWajah.detectMultiScale(abuAbu, scaleFactor = 1.3, minNeighbors = 2) 

        for (x, y, w, h) in dafWajah:
            cv2.rectangle(kerangka, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
        #print("Jumlah Wajah terdeksi: ", len(dafWajah))
        teks = "Jumlah Wajah Terdeteksi = " + str(len(dafWajah))

        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(kerangka, teks, (0, 30), font, 1, (255, 0, 0), 1)

        cv2.imshow("Hasil", kerangka)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            tombolQditekan = True
            break


videoCam.release()
cv2.destroyAllWindows()