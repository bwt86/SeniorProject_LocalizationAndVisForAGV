"""
Calibrates all cameras in system

This script essenially:\n
1. Opens a file to write text
2. For each camera (number of cameras given by operator):
    1. Turns on Cameras
    2. Captures 60 frames and uses the last one to get the data needed for calibration
    
"""
#import all libraries required
import cv2
import numpy as np
from pyzbar.pyzbar import decode
if __name__ == '__main__': 
    #open file calibration data will be written to
    cal = open('cal.txt', 'w')
    #how many cameras are in the system and need to be calibrated
    numCameras = 2;

    #read in the qr code that will be used for calibration
    img = cv2.imread('QrTest.png')

    #get the data for and about the QR code
    for i in decode(img):
        print(i.rect)
        print(i.data.decode('utf-8'))

    #for each camera that will be calibrated (note this assumes cameras are sequential starting from camera 0)
    for k in range(numCameras):

        #set up camture for camera k
        cap = cv2.VideoCapture(k)
        cap.set(3,640)
        cap.set(4,480)

        #variable x used to set number of calibration frames captured
        x=0
        while x < 60:
            #capture each frame
            success, img = cap.read()
            for i in decode(img):
                #for each frame read the data about the code and put it into an array called pts
                print(i.rect)
                print(i.data.decode('utf-8'))
                pts = np.array([i.polygon], np.int32)
                pts = pts.reshape((-1,1,2))
                pts2 = i.rect
                x+=1

        #distance from camera to calibration code for calibration
        CAL_DISTANCE = 24.0

        #width of the calibration code
        KNOWN_WIDTH = 7.0

        #determine focal length of camera k
        focalLength = ( pts2[3] * CAL_DISTANCE) / KNOWN_WIDTH
        # write focal length to calibration file
        cal.write(f'{str(focalLength)},', "a")
        #release camera k
        cap.release()
    #close file
    cal.close()