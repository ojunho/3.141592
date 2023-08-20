#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
import time

from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
from detection_msgs.msg import BoundingBoxes
from obstacle_detector.msg import Obstacles

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

from SlideWindow import SlideWindow
from Warper import Warper
from Ultrasonic import Ultrasonic

bridge = CvBridge()
warper = Warper()
slideWindow = SlideWindow()
ultra = Ultrasonic()
motor = None
flag = False

last_x_location = 320

cv_image = np.empty(shape=[0])

CAM_FPS = 30    # 카메라 FPS - 초당 30장의 사진을 보냄
WIDTH, HEIGHT = 640, 480    # 카메라 이미지 가로x세로 크기

MODE = 1

# for ar tag
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
ar_roll, ar_pitch, ar_yaw = 0, 0, 0
k = 2
k1 = 3
k2 = 20
l = 30
l2 = 5
angle_cal = 0.017
angle_cal2 = 0.05
ar_id = None

# for bounding boxes
carLabel = "sonata"
dir = None
carCnt = 0

# for 장애물
mode = None


# 라바콘 클래스
class Rabacon_drive:
    
    def __init__(self):
        rospy.Subscriber("raw_obstacles", Obstacles, self.Rabacon)
        self.angle = 0.0  # Initialize angle as a class attribute
        self.rabacon = False

    def Rabacon(self, data):
        left_rabacon = []
        right_rabacon = []
        print("go")
        for i in data.circles:
            if i.center.x > -1.5 and i.center.x < -0.05:
                if i.center.y > 0 and i.center.y < 0.7:
                    left_rabacon.append(i)
                elif i.center.y < 0 and i.center.y > -0.7:
                    right_rabacon.append(i)
        
        if len(left_rabacon) > 0 and len(right_rabacon) > 0:
            left_close_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)[0]
            right_close_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)[0]
            avg_y_rabacon = (left_close_rabacon.center.y + right_close_rabacon.center.y) 
            self.angle = avg_y_rabacon * 150 # Store angle as a class attribute
            self.rabacon = True
        else:
            print("no_rabacon")
            self.rabacon = False


def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")


#=============================================
# Bounding Boxes 토픽을 발행하는 함수
#=============================================
def boxes_callback(msg):

    global carLabel, dir, carCnt

    xSum = None

    for bounding_box in msg.bounding_boxes:
        if bounding_box.Class == carLabel:
            if bounding_box.probability >= 0.85:
                xSum = bounding_box.xmin + bounding_box.xmax
                carCnt += 1              
    
    if xSum != None and carCnt >= 20:
        if xSum <= 640:
            dir = "left"
        elif xSum > 640:
            dir = "right"
    
    print(carCnt)
    
    if dir != None:
        print(dir)

def ultra_callback(msg):
    global ultra
    ultra.ultra_data(msg.data)

# for ar tag
def ar_callback(msg):
    global ar_id, arData
    for i in msg.markers:
        ar_id = i.id
        pose = i.pose.pose
        arData["DX"] = pose.position.x
        arData["DY"] = pose.position.y
        arData["DZ"] = pose.position.z

        arData["AX"] = pose.orientation.x
        arData["AY"] = pose.orientation.y
        arData["AZ"] = pose.orientation.z
        arData["AW"] = pose.orientation.w

def id_back():
    global ar_id
    return ar_id

def ar_data():
    global k, angle_cal, arData, l
    (ar_roll, ar_pitch, ar_yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    angle = int(k * np.degrees(arData["DX"] - angle_cal)) - l

    return ar_roll, ar_pitch, ar_yaw, angle, arData["DX"], arData["DZ"]

def re_parking():
    global arData
    print(" AR PARKING DETECTED ")
    angle = arData["DX"] * 20
    speed = 0
    parking = False
    if(ultra.ultra_steer()):
        speed = -4
    else:
        speed = 4

    if(-0.05 < arData["DX"] < 0.05):
        parking = True
    
    return angle, speed, parking
        

def esc_p_parking():
    global MODE
    global ar_id

    drive(0, 0)
    time.sleep(3)
    sec = time.time()
    while time.time() - sec <= 0.9:
        drive(-5, -4)
    sec = time.time()
    while time.time() - sec <= 3:
        drive(-50, 4)
    sec = time.time()
    while time.time() - sec <= 1.5:
        drive(30, 4)

    ar_id = None
    

# 장애물 피하기
def decision_Obstacles(data):
    global mode
    left_ob = []
    right_ob = []
    for i in data.circles:
        if i.center.x > -0.4 and i.center.x < -0.0:
            if i.center.y < 0 and i.center.y > -0.4:
                left_ob.append(i)
            elif i.center.y > 0 and i.center.y < 0.4:
                right_ob.append(i)
    
    if len(left_ob) > 0: #왼쪽 장애물
        print("left")
        mode = "left_ob"
    elif len(right_ob) > 0: #오른쪽 장애물
        print("right")
        mode = "right_ob"
    else:
        mode = "lane_drive"


def check_stopline(image):
    area = image[240:380, 185:380]
    stopline_count = cv2.countNonZero(area)
    # print(stopline_count)
    if stopline_count > 1650:
        return "StopLine"
    else:
        return "no"

def drive(angle, speed):

    global motor

    motor_msg = xycar_motor()

    motor_msg.angle = angle
    motor_msg.speed = speed

    motor.publish(motor_msg)

def start():

    global motor
    global cv_image
    global angle_cal
    global MODE
    global flag
    global ar_id
    global last_x_location
    global k2
    global k
    global l2
    global mode


    # count 변수들
    p_parking_cnt = 0


    # flags
    p_parking_flag = 1
    t_parking_flag = 0

    # ar flag
    ar_flag = 0

    # 장애물 우회 주행 flag
    ob_flag = 0

    # 정지선 flag
    stop_flag = 0

    # 라바콘 flag
    rabacon_flag = 0



    # 평행주차 변수들
    # 수정을 하며 앞으로 갈 때

    # 수정을 하며 뒤로갈 때 
    back_first_half = 0.72
    back_second_half = 0.72

    # 처음 진입하며 들어가는 시간들
    p_parking_one_quarter_time = 3
    p_parking_two_quarter_time = 2.6
    p_parking_three_quarter_time = 0
    p_parking_four_quarter_time = 1.9


    # 오른쪽 주차 시간
    # T 주차 변수들
    t_one_r = 0.9
    t_two_r = 3.2
    t_three_r = 0.8
    t_four_r = 3
    t_five_r = 1
    t_six_r = 1.5

    # 왼쪽 주차 시간
    # T 주차 변수들
    t_zero_l = 0.4
    t_one_l = 0.7
    t_two_l = 3.4
    t_three_l = 0.8
    t_four_l = 3
    t_five_l = 1.3
    t_six_l = 1.5

    angle = 0
    DZ = 0
    DX = 0

    # 장애물 변수들
    obstacle_cnt = 0
    direction = 0
    car_speed = 4
    
    rospy.init_node('ar_tag')
    
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_callback,queue_size= 1)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback, queue_size = 1)
    rospy.Subscriber('/yolov5/detections', BoundingBoxes, boxes_callback)

    # 장애물
    rospy.Subscriber('raw_obstacles2', Obstacles, decision_Obstacles)

    rabacon = Rabacon_drive()  # Create an instance of Rabacon_drive class

    print ("----- Xycar self driving -----")
    rospy.wait_for_message("/usb_cam/image_raw/", Image)

    while not rospy.is_shutdown():
        img = cv_image.copy()

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)

        low_threshold = 70
        high_threshold = 210
        canny = cv2.Canny(blur_gray, low_threshold, high_threshold)
        kernel = np.ones((5, 5), np.uint8)
        close = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)
        warp_img = warper.warp(close)
        ret, thres_img = cv2.threshold(warp_img, 150, 255, cv2.THRESH_BINARY)

        cv2.imshow("original_img", img)
        cv2.imshow("processed_img", thres_img)
        cv2.waitKey(1)

        slide_img, x_location, _ = slideWindow.slidewindow(thres_img,  flag)
        x_loc = x_location

        if x_location == None:
            flag = True
            x_location = last_x_location
        
        else:
            last_x_location = x_location
            # if flag == True and x_location > 279:
            #     x_location += 13
            flag = False
        
        cv2.imshow("window_view", slide_img)
        cv2.waitKey(1)

        ar_id = id_back()


        # # 차선주행 코드.
        # if MODE == 0:
        #     angle = int((320 - int(x_location))*-1)
        #     # speed = max(5, 40 - abs(angle))
        #     speed =0
        #     drive(angle, speed)

        # MODE에 대한 생각 
        # 0: 기본모드(차선인식 주행)
        # 1: ar_tag 평행주차
        # 2: ar_tag 우회주행
        # 3: T 주차 객체인식 전
        # 4: 장애물
        # 5: 횡단보도
        # 6: 라바콘
        #
        ###################

        # 모드를 부여 하는 곳.
        # if ar_id != None:
        roll, pitch, yaw, angle, DX, DZ = ar_data()
        # 여기서 ar_id에 따라서 MODE를 부여하는 것도 괜찮을듯..
        # if ar_id == 0:
        #     pass
        # else:
        #     ar_id = None


        # MODE 부여 계
        ########################################################################
        # elif ar_id == 1 and 0 < DZ <= 0.7:
        #     MODE = 1
        if 1 <= p_parking_flag <= 5:
            MODE = 1

        # elif 3 <= ar_id <= 8 and 0 < DZ <= 1 and p_parking_flag == 5:
        #     MODE = 2
        elif 1 <= ar_flag <= 3:
            MODE = 2

        # 우회 주행 끝난 판정으로 진입하면 안됨.
        elif 1 <= t_parking_flag <= 4:
            MODE = 3
        # MODE 4 진입은 밑에 main IF 에서 T 주차 끝나고  ob_flag == 1 로 바꿔줌.
        elif 1 <= ob_flag <= 2:
            MODE = 4
        # elif 1 <= obstacle_cnt < 3:
        #     MODE = 4

        # MODE 5 진입은 밑에 main IF 에서 장애물 회피 끝나고  stop_flag == 1 로 바꿔줌.
        elif stop_flag == 1:
            MODE = 5

        elif rabacon_flag == 1:
            MODE = 6
        ########################################################################

                

        # p_parking_flag: parallel parking, 즉 평행 주차를 했는지 안했는 지에 대한 여부
        # 1: 진행중 차선인식 + 주차로 넘어가기 위한 조건 확인
        # 2: 처음 실행되어 뒤로 가는 코드 
        # 3: re-parking 앞으로
        # 4: re-parking 뒤로
        # 5: 가라주차코드
        # 6: 평행주차 끝


        if MODE == 1:
            if p_parking_flag == 1:
                # ar 인식 전까지 차선주행
                if ar_id == 1 and 0 < DZ <= 0.7:
                    sec_p = time.time()
                    p_parking_flag = 2
                else:
                    angle = int((320 - int(x_location))*-1)
                    speed = 4
                    drive(angle, speed)
            
            # 처음 실행되어 뒤로 가는 코드
            elif p_parking_flag == 2:
                sec = time.time()
                if sec - sec_p <= p_parking_one_quarter_time:
                    drive(-5, 4)
                elif sec - sec_p <= p_parking_one_quarter_time + p_parking_two_quarter_time:
                    drive(50, -4)
                elif sec - sec_p <= p_parking_one_quarter_time + p_parking_two_quarter_time + p_parking_three_quarter_time:
                    drive(-5, -4)
                elif sec - sec_p <= p_parking_one_quarter_time + p_parking_two_quarter_time + p_parking_three_quarter_time + p_parking_four_quarter_time:
                    drive(-50, -4)
                else:
                    p_parking_flag = 3
                    sec_p = time.time()
            
            # reparking 앞으로
            elif p_parking_flag == 3:

                sec = time.time()

                # 가라 주차로 넘어가는 코드
                if p_parking_cnt == 3:
                    p_parking_flag = 5
                    sec_p = time.time()
                    continue

                # 너무 앞으로 갔을 때 예외 처리.(DZ 너무 가깝거나 일정 시간 지나면)
                if sec - sec_p > 2:
                    p_parking_flag = 4
                    sec_p = time.time()
                    continue
                
                if 0.051 <= DX <= 0.059:
                    angle = -5
                else:
                    angle = int(k1 * np.degrees(DX - angle_cal2)) - l2

                drive(angle, 4)
            
            # reparking 뒤로
            elif p_parking_flag == 4:

                sec = time.time()

                if sec - sec_p > 2:
                    p_parking_cnt += 1
                    p_parking_flag = 3
                    sec_p = time.time()
                    continue

                if 0.051 <= DX <= 0.059:
                    angle = -5
                    drive(angle, -4)

                else:
                    if sec - sec_p <= back_first_half:
                        angle = int(k2 * np.degrees(DX - angle_cal2)) - l2
                        drive(angle, -4)
                    elif sec - sec_p <= back_first_half + back_second_half:
                        angle = int(-1*k2 * np.degrees(DX - angle_cal2)) - l2
                        drive(angle, -4)
            
            # 가라 주차 코드
            elif p_parking_flag == 5:
                sec = time.time()
                if sec - sec_p <= 0.6:
                    if 0.051 <= DX <= 0.059:
                        angle = -5
                    else:
                        angle = int(k1 * np.degrees(DX - angle_cal2)) - l2
                    drive(angle, 4)
                else:
                    esc_p_parking()
                    p_parking_flag = 6
                    ar_flag = 1
                    continue
                    
        
        # AR 우회주행
        # 1 : 차선주행 + ar_우회주행 조건 확인
        # 2 : ar 우회주행 + 빠져나옴
        # 3 : 객체 인식 하기 좋은 위치 세워주기
        # 4 : ar 우회주행 끝
        elif MODE == 2:
            if ar_flag == 1:
                if 3 <= ar_id <= 8 and 0 < DZ <= 1 and p_parking_flag == 6:
                    ar_flag = 2
                else:
                    angle = int((320 - int(x_location))*-1)
                    speed = 4
                    drive(angle, speed)
            # ar_우회주행
            elif ar_flag == 2:
                #마지막 ar_tag 빠져나올때
                if ar_id == 7 and 0 < DZ <= 0.6:
                    sec = time.time()
                    while time.time() - sec <= 1:
                        drive(-50, 4)
                    ar_flag = 3
                    ar_id = None
                    sec_ar = time.time()
                #아니면 ar_우회주행
                else :
                    angle = angle
                    speed = 4
                    # print("AR DRIVING")
                    # print(f"DX: {DX}   angle: {angle}")
                    drive(angle, speed)
            # 객체 인식 하기 쉬운 위치에 세워주기 pt_flag 지우기
            elif ar_flag == 3:
                sec = time.time()
                if sec - sec_ar <= 3: # 이시간 바꿔 주빈아
                    angle = int((320 - int(x_location))*-1)
                    # speed = max(5, 40 - abs(angle))
                    speed = 4
                    drive(angle, speed)
                else:
                    t_parking_flag = 1
                    ar_flag = 4



        # 객체인식 T 주차
        # 1: 멈춰서 객체 인식 + 방향 결정
        # 2: AR 조건 전까지 차선 주행 + AR tag 조건 확인
        # 3: AR 인식 후 직진 하드 코딩
        # 4: 방향별 T 주차 하드코딩
        # 5 : T 주차 끝
        # idea : 곡선 차선 이탈 방지 장애물 때문에 일정시간 차선 주행 후 모드 바꿔주기 고려
        elif MODE == 3:
            # 1 : 멈춰서 객체 인식 후 방향 결정
            if t_parking_flag == 1:
                if dir != None:
                    t_parking_flag = 2
                else:
                    drive(0, 0)
            # 2 : AR 조건 전까지 차선 주행 + AR tag 조건 확인
            elif t_parking_flag == 2:
                if 0 < DZ <= 0.6:
                    # print("직진 시작")
                    t_parking_flag = 3
                    sec_t = time.time()
                else:
                    angle = int((320 - int(x_location))*-1)
                    speed = 4
                    drive(angle, speed)
            # 3 : AR 인식 후 직진 하드 코딩
            elif t_parking_flag == 3:
                sec = time.time()
                # 직진 하는 시간
                if sec - sec_t <= 6.8:
                    angle = int((320 - int(x_location))*-1)
                    speed = 4
                    drive(angle, speed)
                else:
                    t_parking_flag = 4
                    sec_t = time.time()
            # 4 : 방향별 T 주차 하드코딩
            elif t_parking_flag == 4:
                # 왼쪽
                if dir == "left":
                    sec = time.time()
                    if sec - sec_t <= t_zero_l:
                        drive(-5, 4)
                    if sec - sec_t <= t_zero_l + t_one_l:
                        drive(25, 4)
                    elif sec - sec_t <= t_zero_l + t_one_l + t_two_l:
                        drive(-50, -4)
                    elif sec - sec_t <= t_zero_l + t_one_l + t_two_l + t_three_l:
                        drive(-5, -4)
                    elif sec - sec_t <= t_zero_l + t_one_l + t_two_l + t_three_l + t_four_l:
                        drive(-5, 0)
                    elif sec - sec_t <= t_zero_l + t_one_l + t_two_l + t_three_l + t_four_l + t_five_l:
                        drive(-5, 4)
                    elif sec - sec_t <= t_zero_l + t_one_l + t_two_l + t_three_l + t_four_l + t_five_l + t_six_l:
                        drive(-50, 4)
                    else:
                        t_parking_flag = 5
                        ob_flag = 1
                # 오른쪽
                elif dir == "right":
                    sec = time.time()
                    if sec - sec_t <= t_one_r:
                        drive(-35, 4) 
                    elif sec - sec_t <= t_one_r + t_two_r:
                        drive(40, -4)
                    elif sec - sec_t <= t_one_r + t_two_r + t_three_r: 
                        drive(-5, -4)
                    elif sec - sec_t <= t_one_r + t_two_r + t_three_r + t_four_r:
                        drive(-5, 0)
                    elif sec - sec_t <= t_one_r + t_two_r + t_three_r + t_four_r + t_five_r:
                        drive(-5, 4)
                    elif sec - sec_t <= t_one_r + t_two_r + t_three_r + t_four_r + t_five_r + t_six_r:
                        drive(40, 4)
                    else:
                        t_parking_flag = 5
                        ob_flag = 1
        # 장애물 피하기
        # 1 : 차선 주행 + 장애물 인식 후 방향 결정
        # 2 : 장애물 피하기
        # 3 : 장애물 피하기 끝
        elif MODE == 4:
            if ob_flag == 1:
                # 첫 왼쪽 장애물
                if(obstacle_cnt == 0 and mode == "left_ob"):
                    direction = 1
                    ob_flag = 2
                # 첫 오른쪽 장애물
                elif(obstacle_cnt == 0 and mode == "right_ob"):
                    direction = 2
                    ob_flag = 2
                # 아무것도 안잡히면 차선주행 하기
                else:
                    angle = int((320 - int(x_location))*-1)
                    speed = 4
                    drive(angle, speed)
            # 장애물 피하기 시작 (값도 바꾸기)
            elif ob_flag == 2:
                #LEFT - RIGHT - LEFT
                if(direction == 1):
                    print("LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
                    if mode == "left_ob":
                        obstacle_cnt += 1
                        if obstacle_cnt == 1:
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(30, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(-5, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.6):
                                drive(-40, car_speed)

                        elif obstacle_cnt >= 3:
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(30, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(-5, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.6):
                                drive(-40, car_speed)
                            ob_flag = 3
                            stop_flag = 1
                        continue

                    elif mode == "right_ob":
                        obstacle_cnt += 1
                        if obstacle_cnt == 2:
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(-40, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(-5, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.4):
                                drive(30, car_speed)
                        continue
                #RIGHT - LEFT - RIGHT
                elif direction == 2:
                    print("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR")
                    if mode == "right_ob":
                        obstacle_cnt += 1
                        if obstacle_cnt == 1:
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(-40, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(-5, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.6):
                                drive(30, car_speed)

                        elif obstacle_cnt >= 3:
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(-40, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(-5, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.6):
                                drive(30, car_speed)
                            ob_flag = 3
                            stop_flag = 1
                        continue

                    elif mode == "left_ob":
                        obstacle_cnt += 1
                        if obstacle_cnt == 2:
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(30, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.5):
                                drive(-5, car_speed)
                            sec_L = time.time()
                            while(time.time() - sec_L <= 0.6):
                                drive(-40, car_speed)
                        continue

        # 정지선 검출
        # 1 : 정지선이 있으면, 멈추고 다음 미션으로 넘겨준다.
        # 2 : 정지선 검출 끝
        elif MODE == 5:
            if stop_flag == 1:
                check_stop = check_stopline(thres_img)
                if check_stop == "StopLine":
                    time.sleep(3.5)
                    stop_flag = 2
                    rabacon_flag = 1
                else:
                    angle = int((320 - int(x_location))*-1)
                    speed = 4
                    drive(angle, speed)                
        # 라바콘 우회주행
        # 1 : 라바콘 주행 조건에 부합하면, 라바콘 주행 아니면 차선주행
        # 그냥 끝까지 차선주행하기 고생했다 자이카 차선이탈 하지말고 잘 들어가줘..
        elif MODE == 6:
            if rabacon_flag == 1:
                if rabacon.rabacon == True:
                    # Use the angle value from the Rabacon_drive instance
                    angle_to_drive = int(rabacon.angle)
                    speed = max(8, 15 - abs(angle_to_drive)/3)
                    drive(angle_to_drive, speed)  
                    time.sleep(0.1)  # Example: Sleep for 0.1 seconds to control the update rate
                else:
                    angle = int((320 - int(x_location))*-1)
                    # speed = max(5, 40 - abs(angle))
                    speed = 4
                    drive(angle, speed)


        # #모르겠으면 프린트 찍어보기
        print(f"MODE: {MODE}")
        # print(f"pPark: {p_parking_flag}")
        # print(f"ar_id: {ar_id}")

        # print(f"DX: {DX}")
        # print(f"DZ: {DZ}")
        # print(f"angle: {angle}")

if __name__ == '__main__':
    start()
