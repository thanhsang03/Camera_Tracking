import cv2
from cvzone.FaceDetectionModule import FaceDetector
import pyfirmata
import numpy as np
import serial
import time
import json

cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()


#port = "COM3"
#board = pyfirmata.Arduino(port)
arduino = serial.Serial('COM3', 9600) 
#servo_pinX = board.get_pin('d:9:s') #pin 9 Arduino
#servo_pinY = board.get_pin('d:10:s') #pin 10 Arduino

detector = FaceDetector()
servoPos = [90, 90] # initial servo position

while True:
    success, img = cap.read()
    img =  cv2.flip(img, 1)
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
        #get the coordinate
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        pos = [fx, fy]
        #convert coordinat to servo degree
        servoX = np.interp(fx, [0, ws], [0, 180])
        servoY = np.interp(fy, [0, hs], [180, 0])

        if servoX < 0:
            servoX = 0
        elif servoX > 180:
            servoX = 180
        if servoY < 0:
            servoY = 0
        elif servoY > 180:
            servoY = 180

        servoPos[0] = servoX
        servoPos[1] = servoY


        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2 )
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "Face detected", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3 )

    else:
        cv2.putText(img, "No face detected", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line


    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    #servo_pinX.write(servoPos[0])
    #servo_pinY.write(servoPos[1])

    data1 = {
    "angle_x": servoPos[0] ,
    "angle_y": servoPos[1]
    }

    angle_x = int(servoPos[0])
    angle_y = int(servoPos[1])
    try:
        # data = json.dumps(data)
        # arduino.write(data.encode()) 
        
        # Chuyển hai số nguyên thành một chuỗi và gửi nó qua cổng Serial
        data = f"{angle_x},{angle_y}\n"
        print(data)
        arduino.write(data.encode())    #utf-8  --> byte
        time.sleep(0.1)  
    except KeyboardInterrupt:  # khi ngắt bằng bàn phím
        arduino.close()  

    cv2.imshow("Image", img)
    cv2.waitKey(1)
