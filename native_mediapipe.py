import mediapipe as mp
import cv2
import numpy as np
from cmath import sqrt
import uuid
import os
import socket
import time

# Socket Code-uncomment to communicate with ESP-32
# addres = "B0:B2:1C:0A:DB:46"
# channel = 1
# s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
# s.connect((addres, channel))
# btdiscovery -s"%sc% -%sn%"


mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

cap = cv2.VideoCapture(0)
res_w = 1280
res_h = 720
cap.set(3, res_w)
cap.set(4, res_h)

fingers = [[19, 18, 17, 0], [15, 14, 13, 0], [11, 10, 9, 0], [7, 6, 5, 0]]
joint_list = [[12], [4], [8], [16], [20], [0]]
stop_list = []
stop_thres = 400.0
angle_thres = 15
prev_gear = 0
# global gear
# gear = 0
command = 'start'


def create_point():
    for i in range(len(joint_list)):
        point = np.array([hand.landmark[joint_list[i][0]].x * res_w, hand.landmark[joint_list[i][0]].y * res_h]).astype(
            'int')
        stop_list.append(point)


def tip_distance():
    sum = 0
    stop_list.clear()
    create_point()
    print(len(stop_list))
    for i in range(len(stop_list) - 1):
        x = stop_list[i][0] - stop_list[i + 1][0]
        y = stop_list[i][1] - stop_list[i + 1][1]
        sum += np.abs(sqrt(x * x + y * y))
    sum = round(sum, 2)
    print('Current stop_val:' + str(sum))
    cv2.putText(image, 'Current stop_val:' + str(sum), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1,
                cv2.LINE_AA)
    return sum


def get_direction(hand):
    global gear, command, data
    if tip_distance() > stop_thres:
        # Joints 12 & 0 to determine direction. format: hand.landmark[joint].x
        xa = hand.landmark[12].x * res_w  # a - finger
        ya = hand.landmark[12].y * res_h
        b = np.array([hand.landmark[0].x * res_w, hand.landmark[0].y * res_h]).astype('int')  # b - wrist
        anchor = np.array([b[0], b[1] + 75]).astype('int')  # c - forearm
        radians = np.arctan2(anchor[1] - b[1], anchor[0] - b[0]) - np.arctan2(ya - b[1], xa - b[0])
        angle = radians * 180.0 / np.pi
        cv2.circle(image, (anchor[0], anchor[1]), 4, (121, 22, 250), 2)
        cv2.line(image, (b[0], b[1]), (anchor[0], anchor[1]), (255, 255, 255), 2)
        cv2.putText(image, command, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(image, 'anchor', (anchor[0], anchor[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

        if 140 < angle < 220 and b[1] > ya:
            gear = speed()
            command = 'forward'
            data = 'f'
            print('forward')
        elif 45 > angle > -45:
            command = 'back'
            data = 'b'
            print('back')
        elif 80 < angle < 140 and xa > b[0]:
            command = 'right'
            data = 'r'
            print('right')
        elif (angle > 220 or angle < -65) and xa < b[0]:
            command = 'left'
            data = 'l'
            print('left')
    else:
        print('STOP')
        data = 's'
        cv2.putText(image, 'STOP', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1, cv2.LINE_AA)



def speed():
    global prev_gear
    gear = 0
    for finger in fingers:
        a = np.array([hand.landmark[finger[0]].x, hand.landmark[finger[0]].y])
        b = np.array([hand.landmark[finger[1]].x, hand.landmark[finger[1]].y])
        c = np.array([hand.landmark[finger[2]].x, hand.landmark[finger[2]].y])
        d = np.array([hand.landmark[finger[3]].x, hand.landmark[finger[3]].y])

        rad1 = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
        angle1 = np.abs(rad1 * 180.0 / np.pi)
        rad2 = np.arctan2(d[1] - c[1], d[0] - c[0]) - np.arctan2(b[1] - c[1], b[0] - c[0])
        angle2 = np.abs(rad2 * 180.0 / np.pi)

        if -angle_thres < round(angle1, 1) - round(angle2, 1) < angle_thres:
            gear += 1

    if 1 <= gear <= 3:
        prev_gear = gear
    elif gear == 0 or gear > 3:
        gear = prev_gear

    return gear


def get_angle(image, hand, joint_list):
    for joint in joint_list:
        a = np.array([hand.landmark[joint[0]].x, hand.landmark[joint[0]].y])
        """""
            b = np.array([hand.landmark[joint[1]].x, hand.landmark[joint[1]].y])
            c = np.array([hand.landmark[joint[2]].x, hand.landmark[joint[2]].y])
            radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
            angle = np.abs(radians * 180.0 / np.pi)
            if angle > 180:
                angle = 360 - angle
            """""
        coord = str(tuple(np.multiply(a, [res_w, res_h]).astype(int)))
        cv2.putText(image, coord, tuple(np.multiply(a, [res_w,res_h]).astype(int)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)
    return image


with mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.5, max_num_hands=1) as hands:
    global gear, data
    gear = 0
    data = ''

    while cap.isOpened():

        _, frame = cap.read()
        frame = cv2.flip(frame, 1)
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = hands.process(image)
        image.flags.writeable = True

        if results.multi_hand_landmarks:
            for num, hand in enumerate(results.multi_hand_landmarks):
                mp_drawing.draw_landmarks(image, hand, mp_hands.HAND_CONNECTIONS,
                                          mp_drawing.DrawingSpec(color=(121, 22, 250), thickness=2, circle_radius=4),
                                          mp_drawing.DrawingSpec(color=(200, 200, 250), thickness=2, circle_radius=2)
                                          )

                get_direction(hand)
                cv2.putText(image, 'Speed:' + str(gear), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1,
                            cv2.LINE_AA)

                # Socket Code-uncomment to communicate with ESP-32
                # s.send(bytes(f"{ord(data)},{gear}/\n", "UTF-8"))

                print(f"{ord(data)},{gear}/")
                time.sleep(0.05)
                image = get_angle(image, hand, joint_list)
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        # For saving output frames
        # cv2.imwrite(
        #     os.path.join('output_images', '{}.jpg'.format(uuid.uuid1())),
        #     image
        # )

        cv2.imshow('tracking', image)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            # Socket Code-uncomment to communicate with ESP-32
            # s.close()
            break

cap.release()
cv2.destroyAllWindows()