'''
Author: Arjun Viswanathan
Date created: 9/29/23
Date last modified: 9/30/23
Description: sample script to read from camera using OpenCV-Python
'''

import cv2

path = "E:/UMass_Amherst/SDP/sdp-team-12/"

i = 0

cam = cv2.VideoCapture(0)
cam.set(3, 1920)
cam.set(4, 1080)

while True:
    success, img = cam.read()
    cv2.imshow('rgb', img)

    # Press 'a' key to save image for calibration
    if cv2.waitKey(33) == ord('a'):
        print("Taking pic {}...".format(i))
        cv2.imwrite(path + "Images/image_{}.png".format(i), img)
        i += 1

    if cv2.waitKey(1) == 27:
        break

cv2.destroyAllWindows()