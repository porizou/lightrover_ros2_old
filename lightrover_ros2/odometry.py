#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、ライトローバーのオドメトリを取得するためのノードです。

import rclpy
import sys
from lightrover_ros2.srv import Wrc201Msg
import time
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion ,TransformStamped
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry

MS32_M_POS0 = 0x60
MS32_M_POS1 = 0x64

pre_count = [0.0, 0.0]

diff_count = [0, 0]

DIFF_COUNT_LIMIT = 1048575

pre_time = time.time()
diff_time = 0

WHEEL_CIRCUMFERENCE = 60.0*math.pi/1000
ENC_COUNTS_PER_TURN = 1188.024
ENC_PER_M = ENC_COUNTS_PER_TURN/WHEEL_CIRCUMFERENCE

ROVER_D = 0.143/2.0

x = 0.0
y = 0.0
th = 0.0

rclpy.init(args=sys.argv)
node = rclpy.create_node('wrc201_odometry')
node.get_logger().info('Start calculate odometry')

read_enc = node.create_client(Wrc201Msg, 'wrc201_i2c')

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

def getEncVal():

        try:
                #VS-WRC201上のマイコンからエンコーダ値を取得
                enc_a = read_enc(MS32_M_POS0,0,4,'r')
                enc_b = read_enc(MS32_M_POS1,0,4,'r')

                return enc_a.readData,enc_b.readData

        except rospy.ServiceException as e:
                rospy.loginfo('Service call failed: %s',e)

def calSpeed():
        global diff_time
        global pre_time
        global diff_count
        now_time = time.time()
        diff_time = now_time-pre_time
        pre_time = now_time

        global pre_count

        enc_val = getEncVal()
        if enc_val is None:
                print('None')
                return None

        #以前のエンコーダ値と現在のエンコーダ値の差を算出
        for i in range(2):
                if(abs(enc_val[i]-pre_count[i]) < DIFF_COUNT_LIMIT):
                        diff_count[i] = -1.0 * (enc_val[i]-pre_count[i])

        pre_count = enc_val

        #各タイヤの移動距離を算出
        distance = [float(diff_count[0])/ENC_PER_M,float(diff_count[1])/ENC_PER_M]
        #各タイヤの回転速度を算出
        speed = [distance[0]/diff_time, distance[1]/diff_time]

        #本体の直進・旋回速度を算出
        linear_x = ((speed[0] - speed[1])/2.0)
        angular_z = -1.0 * ((speed[0] + speed[1])/(2.0*ROVER_D))

        return linear_x, angular_z

def cal_odometry(vx, vth):
        global x,y,th
        delta_x = vx*math.cos(th)*diff_time
        delta_y = vx*math.sin(th)*diff_time
        delta_th = vth*diff_time

        x += delta_x
        y += delta_y
        th += delta_th

def lightrover_odometry():
        odom_pub = node.create_publisher(Odometry, 'odom', 50)
        odom_br = TransformBroadcaster(node)

        rate = node.create_rate(2)

        get_val = calSpeed()

        while not rclpy.ok():
                get_val = calSpeed()
                if(get_val is None):
                        continue
                else:
                        cal_odometry(get_val[0],get_val[1])

                        now_time = node.get_clock().now().to_msg()

                        odom_tf = TransformStamped()
                        odom_tf.header.stamp = now_time
                        odom_tf.header.frame_id = "odom"
                        odom_tf.child_frame_id = "base_link"

                        odom_quat = quaternion_from_euler(0, 0, th)

                        odom_tf.transform.translation.x = x
                        odom_tf.transform.translation.y = y
                        odom_tf.transform.translation.z = 0
                        odom_tf.transform.rotation = odom_quat

                        odom_br.sendTransform(odom_tf)

                        odom = Odometry()
                        odom.header.stamp = now_time

                        odom.header.frame_id = "odom"

                        odom.pose.pose.position.x = x
                        odom.pose.pose.position.y = y
                        odom.pose.pose.position.z = 0
                        odom.pose.pose.orientation.x = odom_quat[0]
                        odom.pose.pose.orientation.y = odom_quat[1]
                        odom.pose.pose.orientation.z = odom_quat[2]
                        odom.pose.pose.orientation.w = odom_quat[3]

                        odom.child_frame_id = "base_link"
                        odom.twist.twist.linear.x = get_val[0]
                        odom.twist.twist.linear.y = 0.0
                        odom.twist.twist.angular.z = get_val[1]

                        odom_pub.publish(odom)

                        rate.sleep()

if __name__=="__main__":
        lightrover_odometry()
