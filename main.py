#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from dynamixel_sdk import *


class Subscriber(Node):

    def __init__(self):
        super().__init__('motor_ros2')
        print("3-1. node, obj, sub created...\n")
        self.subscription = self.create_subscription(Joy, 'joystick', self.joystick_callback, 10)
        self.subscription

    def joystick_callback(self, msg):
        global btn4, btn5, btn6, btn7, btn12, axes2, axes3
        global dxl_goal_velocity_motor1, dxl_goal_velocity_motor2
        global dxl_goal_current_motor3, dxl_goal_current_motor4, dxl_goal_current_motor5, dxl_goal_current_motor6
        global exit_program

        axes2 = msg.axes[2] # 카메라1 조이스틱 좌우
        axes3 = msg.axes[3] # 카메라2 조이스틱 상하
        btn12 = msg.buttons[12] # 카메라1,2 초기화
        btn4 = msg.buttons[4] # 굴삭기1 감소
        btn5 = msg.buttons[5] # 굴삭기1 증가
        btn6 = msg.buttons[6] # 굴삭기2 감소
        btn7 = msg.buttons[7] # 굴삭기2 증가

        print("3-2. msg received...\n")
        print(f"카메라1 조이스틱 좌우: {axes2}")
        print(f"카메라2 조이스틱 상하: {axes3}")
        print(f"카메라1,2 초기화 버튼: {btn12}")
        print(f"굴삭기1 감소 버튼: {btn4}")
        print(f"굴삭기1 증가 버튼: {btn5}")
        print(f"굴삭기2 감소 버튼: {btn6}")
        print(f"굴삭기2 증가 버튼: {btn7}\n")

        # 모터 1 속도 조정
        if 1 == btn12:
            dxl_goal_velocity_motor1 = 0
        elif 2 == axes2:
            dxl_goal_velocity_motor1 += VELOCITY_INCREMENT
        elif -2 == axes2:
            dxl_goal_velocity_motor1 -= VELOCITY_INCREMENT

        # 모터 2 속도 조정
        if 1 == btn12:
            dxl_goal_velocity_motor2 = 0
        elif 2 == axes3:
            dxl_goal_velocity_motor2 += VELOCITY_INCREMENT
        elif -2 == axes3:
            dxl_goal_velocity_motor2 -= VELOCITY_INCREMENT

        # 모터3,4 전류 조정
        if btn4 == 1:
            dxl_goal_current_motor3 += CURRENT_INCREMENT
            dxl_goal_current_motor4 += CURRENT_INCREMENT
        elif btn5 == 1:
            dxl_goal_current_motor3 -= CURRENT_INCREMENT
            dxl_goal_current_motor4 -= CURRENT_INCREMENT
        elif btn4 == 1 and btn5 == 1:
            dxl_goal_current_motor3 = 0
            dxl_goal_current_motor4 = 0

        # 모터5,6 전류 조정
        if btn6 == 1:
            dxl_goal_current_motor5 += CURRENT_INCREMENT
            dxl_goal_current_motor6 += CURRENT_INCREMENT
        elif btn7 == 1:
            dxl_goal_current_motor5 -= CURRENT_INCREMENT
            dxl_goal_current_motor6 -= CURRENT_INCREMENT
        elif btn6 == 1 and btn7 == 1:
            dxl_goal_current_motor5 = 0
            dxl_goal_current_motor6 = 0


        self.set_motor(DXL_ID_MOTOR3, DXL_ID_MOTOR4)
        self.set_motor(DXL_ID_MOTOR5, DXL_ID_MOTOR6)
        print("3-3. motor 3-4, 5-6 adjusted...\n")

        self.set_motor(DXL_ID_MOTOR1, dxl_goal_velocity_motor1)
        self.set_motor(DXL_ID_MOTOR2, dxl_goal_velocity_motor2)
        print("3-4. motor 1,2 controlled...\n")

        self.set_motor(DXL_ID_MOTOR3, dxl_goal_current_motor3)
        self.set_motor(DXL_ID_MOTOR4, dxl_goal_current_motor4)
        self.set_motor(DXL_ID_MOTOR5, dxl_goal_current_motor5)
        self.set_motor(DXL_ID_MOTOR6, dxl_goal_current_motor6)
        print("3-5. motor 3,4,5,6 controlled...\n")

        time.sleep(0.1)

# ******************************************************************************************************************************************

    def get_motor(motor_id):
        global exit_program
        dxl_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, motor_id, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS or dxl_error !=0 :
            print(f"get_motor() failed for motor {motor_id}")
            exit_program = True

        return dxl_position

# ******************************************************************************************************************************************

    def set_motor(self, motor_id, value):
        global exit_program

        # 현재 각도 확인
        get_pos = self.get_motor(motor_id)

        # 제한 각도 넘어갔는지 확인하고, 제한 범위내의 각도로 설정
        if (get_pos > MAX_POSITION) or (get_pos < MIN_POSITION):
            position = min(max(get_pos, MIN_POSITION), MAX_POSITION)
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_POSITION, position)

        # 제한 각도내 있다면, 제한 범위내의 속도나 전류로 설정
        else:
            if motor_id ==1 or 2:
                velocity = min(max(value, MIN_VELOCITY), MAX_VELOCITY)
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_VELOCITY, velocity)
            else:
                current = min(max(value, MIN_CURRENT), MAX_CURRENT)
                dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, motor_id, ADDR_GOAL_CURRENT, current)

        # 오류 메세지 출력
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f"set_motor() failed for motor ({motor_id})")
            exit_program = True


# ******************************************************************************************************************************************

    def adjust_motor(self, motor_id1, motor_id2):
        global exit_program, diff_pos_thres

        # 모터의 각도차이값 얻기
        get_pos1 = self.get_motor(motor_id1)
        get_pos2 = self.get_motor(motor_id2)
        diff_pos = abs(get_pos1 - get_pos2)

        # 각도차이가 큰 경우 중간 각도로 일치
        if diff_pos > diff_pos_thres:
            avg_pos = diff_pos//2
            dxl_comm_result1, dxl_error1 = packetHandler.write4ByteTxRx(portHandler, motor_id1, ADDR_GOAL_POSITION, avg_pos)
            dxl_comm_result2, dxl_error2 = packetHandler.write4ByteTxRx(portHandler, motor_id2, ADDR_GOAL_POSITION, avg_pos)

        # 오류 메세지 출력
        if dxl_comm_result1 != COMM_SUCCESS or dxl_error1 !=0 or dxl_comm_result2 != COMM_SUCCESS or dxl_error2 !=0:
            print(f"adjust_motor() failed for motor ({motor_id1}) or ({motor_id2})")
            exit_program = True

# ******************************************************************************************************************************************

MY_DXL = 'X_SERIES'

if MY_DXL == 'X_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_VELOCITY          = 104  # Address for goal velocity
    ADDR_GOAL_CURRENT           = 100
    ADDR_GOAL_POSITION          = 116
    ADDR_PRESENT_VELOCITY       = 128  # Address for present velocity
    ADDR_PRESENT_CURRENT        = 126
    ADDR_PRESENT_POSITION       = 132  # Address for present position
    BAUDRATE                    = 115200

DXL_ID_MOTOR1               = 1  # Motor 1's ID
DXL_ID_MOTOR2               = 2  # Motor 2's ID
DXL_ID_MOTOR3               = 10  # Motor 3's ID
DXL_ID_MOTOR4               = 11  # Motor 4's ID
DXL_ID_MOTOR5               = 12  # Motor 5's ID
DXL_ID_MOTOR6               = 13  # Motor 6's ID
MOTOR_LIST                  = [DXL_ID_MOTOR1, DXL_ID_MOTOR2, DXL_ID_MOTOR3, DXL_ID_MOTOR4, DXL_ID_MOTOR5, DXL_ID_MOTOR6]

DEVICENAME                  = '/dev/ttyUSB1'  # Port configuration
PROTOCOL_VERSION            = 2.0

TORQUE_ENABLE               = 1
TORQUE_DISABLE              = 0
DXL_MOVING_STATUS_THRESHOLD = 20

# TODO: CHECK VALUES
CURRENT_INCREMENT           = 10  # 전류 증감량 설정
VELOCITY_INCREMENT          = 10 # 속도 증감량 설정

# TODO: CHECK VALUES
# "XL430-W250" 모델 참고
# https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-description
MAX_VELOCITY                = 800 #1023
MIN_VELOCITY                = -800 #-1023
MAX_POSITION                = 2047  # 2047은 180도 / 4095는 360도
MIN_POSITION                = 0 # 카메라 최소 회전 각도
MAX_CURRENT                 = 1800  # 2047 시계방향
MIN_CURRENT                 = -1800  # -2047 반시계방향

diff_pos_thres = 100
exit_program = False
btn4, btn5, btn6, btn7, btn12, axes2, axes3 = 0
dxl_goal_velocity_motor1, dxl_goal_velocity_motor2 = 0
dxl_goal_current_motor3, dxl_goal_current_motor4, dxl_goal_current_motor5, dxl_goal_current_motor6 = 0

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    # print("Press any key to terminate...")
    # getch()
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    # print("Press any key to terminate...")
    # getch()
    quit()

# ******************************************************************************************************************************************

def set_torque(motor_id, status):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, status)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"set_torque() failed to ({status}) for motor ({motor_id})")

# ******************************************************************************************************************************************

def motor_sub_thread():
    rclpy.init()
    subscriber = Subscriber()
    rclpy.spin(subscriber)

# ******************************************************************************************************************************************

def main(args=None):
    global exit_program
    global btn4, btn5, btn6, btn7, btn12, axes2, axes3
    global MOTOR_LIST
    print("1. main started...\n")

    # 모터 토크 ENABLE
    for motor in MOTOR_LIST:
        set_torque(motor, TORQUE_ENABLE)
    print("2. motor enabled...\n")

    sub_thread = threading.Thread(target=motor_sub_thread)
    sub_thread.daemon = True
    print("3. thread started...\n")

    sub_thread.start()
    sub_thread.join()
    print("4. thread finished...\n")

    # 모터 토크 DISABLE
    for motor in MOTOR_LIST:
        set_torque(motor, TORQUE_DISABLE)
    print("5. motor disabled...")

    portHandler.closePort()
    rclpy.shutdown()
