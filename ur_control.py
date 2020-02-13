# UR10 control in V-REP simulator
import imutils
import sim
import time, math
# image processing
import numpy as np
import cv2
from kinematics import inv_kin

KOEF = [-0.8380, 0.3498, 0.1890]

rad = math.radians
JOINT_NO = 6

if __name__ == '__main__':
    def nothing(*arg):
        pass


# just in case, close all opened connections
sim.simxFinish(-1)

################################ Connect to V-REP ############################

clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # port is in remoteApiConnections.txt
if clientID != -1: print('Connected to remote API server')

################################## Get handles ###############################

err, cameraID = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_oneshot_wait)
if err == -1: print("No camera")

jointID = [-1, -1, -1, -1, -1, -1]
for i in range(JOINT_NO):
    err, h = sim.simxGetObjectHandle(clientID, 'UR10_joint' + str(i + 1), sim.simx_opmode_oneshot_wait)
    if err == -1:
        print("No joint", i)
    else:
        jointID[i] = h
    time.sleep(0.1)

err, gripperID = sim.simxGetObjectHandle(clientID, 'GRIPPER', sim.simx_opmode_oneshot_wait)
if err == -1: print('No gripper')

############################# Configure and run #############################

# initial position
#initial = [0,0,rad(-90),0,rad(90),0]
initial = [0,rad(45),rad(-90),rad(-15),rad(90),0]
#initial = [0, 0, rad(90), rad(90), rad(90), 0]

# start simulation
sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)

# go to initial position
for i in range(JOINT_NO):
    sim.simxSetJointTargetPosition(clientID, jointID[i], initial[i], sim.simx_opmode_oneshot_wait)

time.sleep(1)  # wait for finishing the previous operation

######################### Example: position / orientation #############################

err, pos = sim.simxGetObjectPosition(clientID, gripperID, -1, sim.simx_opmode_oneshot_wait)
if err == -1: print("Can't get position")
print("Position:", pos)

err, orient = sim.simxGetObjectOrientation(clientID, gripperID, -1, sim.simx_opmode_oneshot_wait)
if err == -1: print("Can't get orientation")
print("Orientation:", orient)

############################# Example: image reading ##################################

# open streaming
err, resolution, image = sim.simxGetVisionSensorImage(clientID, cameraID, 0, sim.simx_opmode_streaming)
# read image
time.sleep(0.2)
MIN_AREA = 110

firstFrame = None
prevCenter = None
movement = None

cv2.namedWindow("result")  # создаем главное окно
vector = [0, 0]
while True:

    err, resolution, image = sim.simxGetVisionSensorImage(clientID, cameraID, 0, sim.simx_opmode_buffer)
    # convert byte array to image
    newImg = np.array(image, dtype=np.uint8)
    newImg.resize([resolution[0], resolution[1], 3])
    # resize the frame, convert it to grayscale, and blur it
    hsv = cv2.cvtColor(newImg, cv2.COLOR_RGB2HSV)
    minColor = np.array((0, 84, 73), np.uint8)
    maxColor = np.array((164, 255, 255), np.uint8)
    tresh = cv2.inRange(hsv, minColor, maxColor)
    cv2.imshow("result", tresh)
    frame = tresh
    blur = cv2.GaussianBlur(frame, (21, 21), 0)

    if firstFrame is None:
        firstFrame = blur
        continue

    # compute the absolute difference between the current frame and
    # first frame
    frameDelta = cv2.absdiff(firstFrame, blur)
    thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

    thresh = cv2.dilate(thresh, None, iterations=2)
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    ccAp = 0
    for c in cnts:
        # if the contour is too small, ignore it
        if cv2.contourArea(c) == ccAp:
            theta = inv_kin(
                [pos[0] + KOEF[0] + vector[0] * 0.001, pos[1] + KOEF[1] - vector[1] * 0.001, pos[2] + KOEF[2], 0,
                 0, 1e-100])
            for i in range(JOINT_NO):
                #sim.simxSetJointTargetPosition(clientID, jointID[i], theta[i], sim.simx_opmode_oneshot_wait)
                pass
            print('min')
            vector = [0, 0]
            ccap = cv2.contourArea(c)
            continue
        ccap = cv2.contourArea(c)
        (x, y, w, h) = cv2.boundingRect(c)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        center = (int(x + w / 2), int(y + h / 2))
        cv2.circle(frame, (int(x + w / 2), int(y + h / 2)), radius=3, color=(0, 255, 0), thickness=-1)
        if prevCenter is not None:
            movement = (center[0] - prevCenter[0], center[1] - prevCenter[1])
            print(movement)
            #print("center = " + str(center))
            prevCenter = center
            vector[0] += movement[0]
            vector[1] += movement[1]

            err, pos = sim.simxGetObjectPosition(clientID, gripperID, -1, sim.simx_opmode_oneshot_wait)
            if err == -1:
                print("Can't get position")

            stale_pos = [-1.3570, 0.1878, -0.1400]
            theta = inv_kin(
                [pos[0] + KOEF[0] + movement[0] * 0.0007, pos[1] + KOEF[1] - movement[1] * 0.0007, pos[2] + KOEF[2], 0,
                 0, 1e-100])
            firstFrame = None
            for i in range(JOINT_NO):
                #sim.simxSetJointTargetPosition(clientID, jointID[i], theta[i], sim.simx_opmode_oneshot_wait)
                pass
        else:
            theta = inv_kin(
                [pos[0] + KOEF[0] + vector[0] * 0.0007, pos[1] + KOEF[1] - vector[1] * 0.0007, pos[2] + KOEF[2], 0,
                 0, 1e-100])
            for i in range(JOINT_NO):
                #sim.simxSetJointTargetPosition(clientID, jointID[i], theta[i], sim.simx_opmode_oneshot_wait)
                pass
            prevCenter = center
    if cnts == []:
        print('stop')
    #cv2.imshow("result", tresh)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
cv2.destroyAllWindows()
# save
# cv2.imwrite('image.png', newImg)
# print ('Image is saved')

############################### Finish work ############################################
sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
sim.simxFinish(clientID)
