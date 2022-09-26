#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーを位置制御するためのノードです。

import rclpy
import sys
from lightrover_ros2.srv import Wrc201Msg
import time
import math
import vs_wrc201_motor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rclpy.init(args=sys.argv)
node = rclpy.create_node('pos_controller')
node.get_logger().info('Start POS Controll')

write_msg = node.create_client(Wrc201Msg, 'wrc201_i2c')
while not write_msg .wait_for_service(timeout_sec=1.0):
    node.get_logger().info('service not available, waiting again...')

#メモリマップアドレス
MU8_O_EN = 0x10
MU8_TRIG = 0x11
MS16_FB_PG0 = 0x20
MS16_FB_PG1 = 0x22

MS32_A_POS0 = 0x48
MS32_A_POS1 = 0x4c

MS16_T_OUT0 = 0x50
MS16_T_OUT1 = 0x52

MU16_FB_PCH0 = 0x30
MU16_FB_PCH1 = 0x32

liner_x = 0.0
angular_z = 0.0

current_v = [0.0, 0.0]
target_rover_v = [0.0, 0.0]

#車輪間距離の半分
ROVER_D = 0.143/2.0

motor_controller = vs_wrc201_motor.VsWrc201Motor()

def cb_get_rover_v(data):
        global linear_x,angular_z,current_v,ROVER_D,target_rover_v
        linear_x = data.twist.twist.linear.x
        angular_z = data.twist.twist.angular.z

        #現在の直進・旋回速度から左右モータの現在の回転速度を算出
        current_v[1] = (linear_x + ROVER_D * angular_z)
        current_v[0] = -1.0 * (linear_x - ROVER_D * angular_z)

        #現在のモータ回転速度と目標のモータ回転速度をPOSコントローラに入力
        #左右モータへの出力値を算出
        output = motor_controller.pos_controll(current_v, target_rover_v)
        drive_motor(output[0],output[1])

def cb_set_target_v(data):
        global ROVER_D,target_rover_v
        #目標直進・旋回速度から左右モータの目標回転速度を算出
        target_rover_v[1] = (data.linear.x + ROVER_D * data.angular.z)
        target_rover_v[0] = -1.0 * (data.linear.x - ROVER_D * data.angular.z)

def drive_motor(r_speed, l_speed):
        write_msg(MS32_A_POS0,r_speed,4,'w')
        write_msg(MS32_A_POS1,l_speed,4,'w')
        write_msg(MU8_TRIG,0x03,1,'w')

def pos_cntrl():
        write_msg(MU8_O_EN,0x00,1,'w')          #モータ出力禁止
        write_msg(MU8_TRIG,0x0c,1,'w')          #エンコーダリセット
        write_msg(MS16_FB_PG0,0x0080,2,'w')     #モータ0位置補償Pゲイン設定
        write_msg(MS16_FB_PG1,0x0080,2,'w')     #モータ1位置補償Pゲイン設定
        write_msg(MU16_FB_PCH0,0x09C4,2,'w')    #モータ0最低出力値設定
        write_msg(MU16_FB_PCH1,0x09C4,2,'w')    #モータ1最低出力値設定
        write_msg(MU8_O_EN,0x03,1,'w')          #モータ出力許可

        node.create_subscription(Odometry, 'odom', cb_get_rover_v)
        node.create_subscription(Twist, 'rover_drive', cb_set_target_v)
        rclpy.spin(node)

if __name__ == '__main__':
        pos_cntrl()
