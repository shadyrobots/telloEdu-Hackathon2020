import cv2, time
import matplotlib.pyplot as plt
import cvlib as cv
from cvlib.object_detection import draw_bbox
from djitellopy import Tello
import numpy as np
from datetime import datetime, timedelta
import time
log = ["System started at {}".format(datetime.now())]

t1 = datetime.now() # initialize 'timer'
waitTime = 5 #seconds

patrolloops = 1
patrolCommands = ["forward 25",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "ccw 30",
                  "ccw 30",
                  "ccw 30",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "ccw 30",
                  "ccw 30",
                  "ccw 30",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "ccw 30",
                  "ccw 30",
                  "ccw 30",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "forward 25",
                  "ccw 30",
                  "ccw 30",
                  "ccw 30",]
i = 0
j = 1

def resetTimer():
    t1 = datetime.now()

w, h = 640, 480
processImages = True
flyMission = True
inFlight = False

def in2cm(inches):
    return round(inches*2.54)

def missionPadXYZ():
    mpID = tello.get_mission_pad_id()
    if mpID == -1:
        return mpID, -1, -1, -1 # doh, nothing to see here
    else:
        x = tello.get_mission_pad_distance_x()
        y = tello.get_mission_pad_distance_y()
        z = tello.get_mission_pad_distance_z()
        return mpID, x, y, z

def textOnFrame(frame, texts):
    h, w, c = frame.shape
    h_offset = h - 5
    for text in texts:
        cv2.putText(frame, text, (5, h_offset),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        h_offset = h_offset - 15

def bboxXY(box):
    x = round(box[0] + ((box[2]-box[0])/2))
    y = round(box[1] + ((box[3]-box[1])/2))
    return (x,y)

def addLaser(frame, target):
    h, w, c = frame.shape
    frame = cv2.line(frame,(round(w*(1/3)),h),bboxXY(bbox[target]),(255,0,0),5)
    frame = cv2.line(frame,(round(w*(2/3)),h),bboxXY(bbox[target]),(255,0,0),5)
    return frame

tello = Tello()
tello.connect()
tello.enable_mission_pads()
tello.set_mission_pad_detection_direction(2)
tello.streamon()

movie = cv2.VideoWriter('filename.avi',  
                         cv2.VideoWriter_fourcc(*'MJPG'), 
                         10, (640,480)) 

while True:
    img = cv2.resize(tello.get_frame_read().frame,(w,h))
    if processImages == True:
        bbox, label, conf = cv.detect_common_objects(img)
        output_image = draw_bbox(img, bbox, label, conf)
        if 'laptop' in label:
            log.append('{}: Laptop identified, fire the lasers!'.format(datetime.now()))
            index = label.index('laptop')
            output_image = addLaser(output_image,index)
        if 'dog' in label:
            log.append('{}: Dog identified...woof'.format(datetime.now()))
    else:
        output_image = img

    mpID, x, y, z = missionPadXYZ()
    if mpID == -1:
        mp_str = "404: Mission Pad Not Found"
    else:
        mp_str = "Mission Pad:{} at X:{} Y:{} Z:{}".format(mpID, x, y, z)

    bat_str = "Battery: {}%".format(tello.get_battery())
    temp_str = "Temperature: {} F".format(round(tello.get_temperature()*(9/5)+32),1)
    alt_str = "Altitude: {}cm".format(tello.get_distance_tof())
    textOnFrame(output_image, [bat_str, temp_str, alt_str, mp_str])

    movie.write(output_image)
    cv2.imshow('Drone TV', output_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        tello.streamoff()
        cv2.destroyAllWindows()
        break

    if flyMission == True and inFlight == False:
        tello.takeoff()
        inFlight = True
    if flyMission == True and inFlight == True:
        t2 = datetime.now()

        delta = t2 - t1
        #print(delta.seconds)
        if delta.seconds > waitTime:
            for retry in range(3):
                ret = tello.send_command_with_return(patrolCommands[i])
                print("{}: {}".format(ret, patrolCommands[i]))
                if ret == "ok":
                    break
                else:
                    print("retry")
                if retry == 3:
                    tello.land()
            i = i+1
            t1 = datetime.now()
        if i >= len(patrolCommands):
            if j < patrolloops:
                i = 0
            else:
                tello.land()
            break
    

tello.streamoff()
cv2.destroyAllWindows()
movie.release()

f = open("flightLog{}.txt".format(datetime.now().strftime("%Y%m%d-%H%M%S")), "w+")
for event in log:
    f.write("{}\n".format(event))
