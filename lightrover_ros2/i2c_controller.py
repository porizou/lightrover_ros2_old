#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#このプログラムは、VS-WRC201を制御するためのノードです。

import rclpy
from std_msgs.msg import String
from lightrover_ros2 import vs_wrc201_i2c
import time
from lightrover_ros2.srv import Wrc201Msg

i2c = vs_wrc201_i2c.VsWrc201I2c(0x10)

def handle_wrc201_i2c(req):
        if(req.cmd=="w"):
                #マイコンのメモリマップの特定アドレスを上書き
                try:
                        if(req.length==4):#4byte
                                i2c.write_4_byte(req.addr,req.data)
                        elif(req.length==2):#2byte
                                i2c.write_2_byte(req.addr,req.data)
                        elif(req.length==1):#1byte
                                i2c.write_1_byte(req.addr,req.data)
                except IOError as e:
                        return None

                return Wrc201MsgResponse(1)

        elif(req.cmd=="s"):
                #マイコンのメモリマップをすべて上書き
                try:
                        i2c.send_write_map()
                except IOError as e:
                        return None

                return Wrc201MsgResponse(1)

        elif(req.cmd=='rm'):
                #マイコンのメモリマップをすべて読み込み
                try:
                        i2c.read_all()
                except IOError as e:
                        return None

                return Wrc201MsgResponse(1)

        elif(req.cmd=="r"):
                #マイコンのメモリマップの特定アドレスを読み込み
                try:
                        i2c.read_memmap(req.addr,req.length)
                        if(req.length==4):#4byte
                                read=i2c.read_s32map(req.addr)
                        elif(req.length==2):#2byte
                                read=i2c.read_s16map(req.addr)
                        elif(req.length==1):#1byte
                                read=i2c.read_s8map(req.addr)
                        else:
                                read=0
                except IOError as e:
                        return None

                return Wrc201MsgResponse(read)

def wrc201_i2c_server():
        rclpy.init(args=sys.argv)
        node = rclpy.create_node('wrc201_i2c_server')

        p_dev_addr = node.declare_parameter('dev_addr', 0x10)

        i2c.set_dev_addr(p_dev_addr)
        i2c.read_all()
        i2c.init_memmap(2.0)
        i2c.send_write_map()
        s = node.create_subscription(Wrc201Msg,'wrc201_i2c',handle_wrc201_i2c)
        
        node.get_logger().info('Ready to VS-WRC201 i2c communication.')
        rclpy.spin(node)

if __name__ == "__main__":
        wrc201_i2c_server()
