import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
from pyzbar.pyzbar import decode

GPIO.setmode(GPIO.BOARD)
# Set the GPIO pin for the servo motor
servo_pin = 22
GPIO.setup(servo_pin, GPIO.OUT)
# Create a PWM instance
pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz (20 ms period)

# Define the servo range
servo_min = 2.5  # Minimum pulse width (in ms)
servo_max = 12.5  # Maximum pulse width (in ms)
authorized_detected =False
count = 1
flag=True
def angle_to_duty_cycle(angle):
    return (angle / 180.0) * (servo_max - servo_min) + servo_min

def rotate_image(img):
    return cv2.rotate(img ,cv2.ROTATE_90_CLOCKWISE)
def rotate_motor():
    pwm.start(angle_to_duty_cycle(90))
    time.sleep(1)

    # Wait for 10 seconds
    time.sleep(3)

     # Move the servo back to its original position (0 degrees)
    pwm.start(angle_to_duty_cycle(0))
    time.sleep(3)
    return()

#img = cv2.imread('1.png')
cap = cv2.VideoCapture(0)
cap.set(3,640)
cap.set(4,480)


with open('Authorized_vehicle.txt') as f:
    myDataList = f.read().splitlines()

while flag:

    success, img = cap.read()
    img = rotate_image(img)
    img = rotate_image(img)
    img = rotate_image(img)
    for barcode in decode(img) :
        myData = barcode.data.decode('utf-8')
        # print(myData)

        if myData in myDataList:
            myOutput = 'Authorized Vehicle'
            myColor = (0,255,0)
            rotate_motor()
            print(myOutput)
            flag=False
           
        else:
            myOutput = 'Un-Authorized Vehicle'
            print(myOutput)
            myColor = (0, 0, 255)

        pts = np.array([barcode.polygon],np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.polylines(img,[pts],True,myColor,5)
        pts2 = barcode.rect
        cv2.putText(img,myOutput,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX,0.9,myColor,2)

    cv2.imshow('Result',img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    

cap.release()
cv2.destroyAllWindows()