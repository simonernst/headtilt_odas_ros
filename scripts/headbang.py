#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
from dynamixel_sdk import *
import time
import rospy
from geometry_msgs.msg import PoseArray

PROTOCOL_VERSION = 1.0

class TiltHead():

    def __init__(self):
        rospy.init_node('tilthead',anonymous=True)
        self.port = "/dev/ttyACM0"
        self.baudrate = 115200
        self.motor_id = 2

        self.sub = rospy.Subscriber("/odas/sst_poses", PoseArray, self.callback, queue_size=1)

        self.addr_cur_pos = 36
        self.addr_new_pos = 30
        self.new_pos = 0
        self.init_pos = 512
        self.serial = serial.Serial(port= self.port, baudrate=self.baudrate)
        self.portHandler = PortHandler(self.port)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.list = []
        print(self.connect())

        

    def writehead(self, new_position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.motor_id, self.addr_new_pos, new_position)

    def connect(self):
        return self.portHandler.openPort()

    def meanlist(self, list):
        return sum(list) / len(list)


    def callback(self, msg):
        try:
            data = msg.poses[0].orientation.z
        except:
            data = 0    
        
        self.list.append(data)
        if len(self.list) > 100 :
            self.list.pop(0)

        mean_pos = self.meanlist(self.list)            
        position = self.init_pos + int(mean_pos*self.init_pos)

        self.writehead(int(position))

if __name__ == "__main__":
    TiltHead()
    rospy.spin()