'''
Author: Arjun Viswanathan
Date created: 9/25/23
Date last modified: 9/25/23
Description: detecting ArUco markers off a live camera feed
'''

import cv2
import scipy.io as sio

camParams = sio.loadmat("camParams.mat")
cameraMatrix = camParams['cameraMatrix']
distCoeffs = camParams['distortionCoefficients']

camera = cv2.VideoCapture(0)
success = 1

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
markerLength = 142 # mm

print("Reading from camera...\n")

i = 0
j = 0

while success:
    success, image = camera.read()
    s = image.shape

    (corners, ids, rejected) = detector.detectMarkers(image)

    if len(corners) > 0:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

        for (markerCorner, markerID) in zip(corners, ids):
            reshapedCorners = markerCorner.reshape((4, 2))
            (tL, tR, bR, bL) = reshapedCorners
            topLeft = [int(tL[0]), int(tL[1])]
            
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, markerLength, cameraMatrix, distCoeffs)
            rvec = rvec[0][0]
            tvec = tvec[0][0]

            cv2.putText(image, str(round(tvec[2], 2)), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print("Marker detected! ID: {}, RVEC: {}, TVEC: {}".format(str(markerID), rvec, tvec))

    cv2.imshow("ArUCO Detection", image)

    if cv2.waitKey(33) == ord('a'):
        print("Taking pic {}...".format(i))
        cv2.imwrite("Images/image_{}.png".format(i), image)
        i += 1

    if cv2.waitKey(33) == ord('s'):
        print("Taking ArUco pic {}...".format(j))
        cv2.imwrite("Images/aruco_image_{}.png".format(j), image)
        j += 1

    if cv2.waitKey(1) == 27: # ESC key to exit
        break

print("Camera terminated. Finished reading!\n")
cv2.destroyAllWindows()