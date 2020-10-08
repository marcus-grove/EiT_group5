#!/usr/bin/env python

import rospy
from math import pi, radians, degrees, tan, sin, cos
import mavros as mav
import mavros.utils
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (String, Int8, Float64)
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from geometry_msgs.msg import Quaternion

mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'
targetWP = '/onboard/setpoint/loiter'

keySub = '/gcs/command'

debug = True
class loiterPilot(): 
    def __init__(self):
        ''' ROS node init '''
        rospy.init_node('loiterPilot')
        self.rate = rospy.Rate(20)

        ''' General '''
        self.enable = False
        self.uavHead = 0.0
        self.headingMode = False        

        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'),mavSP.PoseStamped, self.onPositionChange)
        rospy.Subscriber(mavros.get_topic('global_position','compass_hdg'), Float64, self.onHeadingUpdate)
        rospy.Subscriber(keySub, Int8, self._cb_onKeypress)
        
        # self.targetPub = rospy.Publisher
        # self.loiterPub = mavSP.get_pub_position_local(queue_size=5)


        
        self.loiterPub = rospy.Publisher(targetWP, mavSP.PoseStamped, queue_size=5)
        self.loiterPos = mavSP.PoseStamped()
        self.curPos = mavSP.PoseStamped()
        
        rospy.loginfo('loiterPilot Ready')

    def _pubMsg(self, msg, topic):
        if self.headingMode:
            f_ID="base_link"
        else:
            f_ID="att_pose"
        msg.header = mavros.setpoint.Header(
            frame_id=f_ID,
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    def adjustYaw(self, angle=5.0): 
        (_, _, yaw) = euler_from_quaternion([self.loiterPos.pose.orientation.x, self.loiterPos.pose.orientation.y, self.loiterPos.pose.orientation.z, self.loiterPos.pose.orientation.w])

        yaw += radians(angle)
        orientAdj = quaternion_from_euler(0, 0, yaw)

        self.loiterPos.pose.orientation = Quaternion(*orientAdj)
        
    def setBearing(self, bearing=0.0):

        orientAdj = quaternion_from_euler(0, 0, radians(bearing))

        desiredBearing = mavSP.PoseStamped()
        desiredBearing.pose.orientation = Quaternion(*orientAdj)
        
        return desiredBearing.pose.orientation

    def _cb_onKeypress(self, msg):
        
        keypress = str(chr(msg.data))
        keypress.lower()
        # print("I got:", keypress)
        if self.enable:
            if keypress == 'd':
                self.loiterPos.pose.position.x += 0.5 
            if keypress == 'a':
                self.loiterPos.pose.position.x -= 0.5
            if keypress == 'w':
                self.loiterPos.pose.position.y += 0.5
            if keypress == 's':
                self.loiterPos.pose.position.y -= 0.5    
            if keypress == 'z':
                self.loiterPos.pose.position.z += 0.5 
            if keypress == 'x':
                self.loiterPos.pose.position.z -= 0.5
            if keypress == 'q':
                self.adjustYaw(5.0)
            if keypress == 'e':
                self.adjustYaw(-5.0)

            # if keypress == 'h':
            #     self.headingMode = not self.headingMode
            #     print("headingMode: ", self.headingMode) 
            if debug: 
                if keypress == 'r':
                    # move the drone to a position on the model power line 
                    rospy.loginfo("Loiter: moving to powerline set. ")
                    self.loiterPos.pose.position.x = 16
                    self.loiterPos.pose.position.y = 5
                    self.loiterPos.pose.position.z = 27
                    self.loiterPos.pose.orientation = self.setBearing(315.0)
                if keypress == 'f':
                    rospy.loginfo("Loiter: moving to single power line")
                    self.loiterPos.pose.position.x = -38.5
                    self.loiterPos.pose.position.y = 24.0
                    self.loiterPos.pose.position.z = 13.0
                    self.loiterPos.pose.orientation = self.setBearing(80)
                if keypress == 'h':
                    #Home position
                    self.loiterPos.pose.position.x = 0
                    self.loiterPos.pose.position.y = 0
                    self.loiterPos.pose.position.z = 7.5

        else:
            options = "wasdqezxh"        #options as above
            if keypress in options:  
                # print("warn: loiterpilot not enabled")
                pass
                
    def onHeadingUpdate(self,msg):
        self.uavHead = msg.data

    def onStateChange(self, msg):
        if msg.data == 'loiter':
            print('loiter enabled')
            self.loiterPos = self.curPos
            # self.loiterPos.pose.orientation = self.setBearing(self.uavHead)
            self.enable = True
        else:
            if self.enable:
                print('loiter disabled')
            self.enable = False
        
    def onPositionChange(self,msg):
        self.curPos = msg

    def run(self):

        while not rospy.is_shutdown():
            if self.enable:
                self._pubMsg(self.loiterPos, self.loiterPub)
                self.rate.sleep()

if __name__ == "__main__":
    LP = loiterPilot()
    LP.run()
