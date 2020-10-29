#!/usr/bin/env python

import rospy
from math import pi, radians, degrees, tan, sin, cos
import mavros as mav
import mavros.utils
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (String, Int8, Float64, Bool)
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from geometry_msgs.msg import Quaternion

import numpy as np

mavros.set_namespace('mavros')

onB_StateSub    = '/onboard/state'
onB_SubStateSub = '/onboard/substate'
targetWP        = '/onboard/setpoint/mission'
wpCompleteTopic = '/onboard/check/WPSuccess'
commandSub      = '/gcs/command'

class missionControl(): 
    def __init__(self):
        ''' ROS node init '''
        rospy.init_node('missionControl')
        self.rate = rospy.Rate(20)

        ''' General '''
        self.enable = False
        self.onState = 'idle'
        self.missionState = 'idle'
        self.waypoint = mavSP.PoseStamped()
        self.forward = True        
        self.wpComplete = False

        self.nextWaypoint = mavSP.PoseStamped()
        self.nextState = 'idle'
        self.nextSubState = 'idle'
        self.nextCommand = 't'

        self.index = 0
   
        ''' Subsribers '''
        # Onboard state
        rospy.Subscriber(onB_StateSub, String, self._cb_onStateChange)

        rospy.Subscriber(wpCompleteTopic, Bool, self._cb_onWPComplete)

        ''' Publishers '''
        # Target waypoint for loiter pilot    
        self.waypointPub = rospy.Publisher(
            targetWP, 
            mavSP.PoseStamped, 
            queue_size=5)
      
        # Onboard State
        self.statePub = rospy.Publisher(
            onB_StateSub,
            String,
            queue_size=1)        

        # Onboard Substate
        self.substatePub = rospy.Publisher(
            onB_SubStateSub,
            String,
            queue_size=1)

        # Control commands
        self.commandPub = rospy.Publisher(
            commandSub,
            Int8,
            queue_size=1)

        rospy.loginfo('Mission: MissionControl Ready')

    ''' Core Functions '''
    def _pubMsg(self, msg, topic):
        f_ID="att_pose"
        msg.header = mavros.setpoint.Header(
            frame_id=f_ID,
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    ''' Functions '''
    # TODO: Implement read mission file
    def executeMission(self):
        if self.missionState == 'idle':
            # load mission
            self.missionState = 'getNextCommand'

        elif self.missionState == 'getNextCommand':
            # get command
            if self.forward == False:
                self.index = self.index - 1
                self.forward = True

            waypoints = [[-5, 0, 1.0], [5, 0, 1.0], [0, 0, 1.0]]
            angle = Quaternion(*quaternion_from_euler(0, 0, np.deg2rad(90)))

            if self.index == 0:
                self.index = 1            
                self.missionState = 'takeoff'
            elif self.index == 1:
                self.nextSubState = 'fence_breach_detection'
                self.index = 3
                self.missionState = 'setSubState'
            elif self.index == 2:
                self.nextState = 'mission'
                self.index = 3
                self.missionState = 'setState'
            elif self.index == 3:
                self.nextWaypoint.pose.position.x = waypoints[0][0]
                self.nextWaypoint.pose.position.y = waypoints[0][1]
                self.nextWaypoint.pose.position.z = waypoints[0][2]
                self.nextWaypoint.pose.orientation = angle
                self.index = 4
                self.missionState = 'setWaypoint'
            elif self.index == 4:
                self.nextWaypoint.pose.position.x = waypoints[1][0]
                self.nextWaypoint.pose.position.y = waypoints[1][1]
                self.nextWaypoint.pose.position.z = waypoints[1][2]
                self.nextWaypoint.pose.orientation = angle
                self.index = 5
                self.missionState = 'setWaypoint'
            elif self.index == 5:
                self.nextWaypoint.pose.position.x = waypoints[2][0]
                self.nextWaypoint.pose.position.y = waypoints[2][1]
                self.nextWaypoint.pose.position.z = waypoints[2][2]
                self.nextWaypoint.pose.orientation = angle
                self.index = 6
                self.missionState = 'setWaypoint'
            elif self.index == 6:
                self.nextCommand = 'l'
                self.nextState = 'land'
                self.missionState = 'sendCommand'
                self.index = 7
            elif self.index == 7:
                self.missionState = 'complete'

           
            # get arguments
            # change state

        elif self.missionState == 'takeoff':
            #print ('sending takeoff command')
            self.commandPub.publish(ord('t'))
            self.missionState = 'waitForTakeoff'

        elif self.missionState == 'waitForTakeoff':
            #print ('wait for takeoff')
            if self.onState == 'loiter':
                self.missionState = 'getNextCommand'
            pass

        elif self.missionState == 'sendCommand':
            self.commandPub.publish(ord(self.nextCommand))
            self.missionState = 'waitForState'

        elif self.missionState == 'waitForState':
            if self.onState == self.nextState:
                self.missionState = 'getNextCommand'

        elif self.missionState == 'setState':
            self.statePub.publish(self.nextState)
            self.missionState = 'getNextCommand'

        elif self.missionState == 'setSubState':
            self.substatePub.publish(self.nextSubState)
            self.missionState = 'getNextCommand'

        elif self.missionState == 'updateParam':
            # update param on PX4
            self.missionState = 'getNextCommand'

        elif self.missionState == 'setWaypoint':
            self.waypoint = self.nextWaypoint

            self.missionState = 'waitForWP'

            if self.onState != 'mission':
                self.forward = False
                self.nextState = 'mission'
                self.missionState = 'setState'
            self.rate.sleep()

        elif self.missionState == 'waitForWP':
            if self.wpComplete == True:
                self.missionState = 'getNextCommand'

        elif self.missionState == 'complete':
            rospy.loginfo('Mission: Mission complete')
            self.index = 0
            self.enable = False
            self.missionState = 'idle'

        
        if self.onState != 'idle':
            self._pubMsg(self.waypoint, self.waypointPub)


    ''' Subscriber callbacks '''
    def _cb_onStateChange(self, msg):
        self.onState = msg.data
        if msg.data == 'mission':
            rospy.loginfo('Mission: Enabled')
            self.enable = True
        elif msg.data == 'idle':
            if self.enable:
                rospy.loginfo('Mission: Disabled')
            self.enable = False

    def _cb_onWPComplete(self,msg):
        self.wpComplete = msg.data
        pass

    #def _cb_onPositionChange(self,msg):
        #self.curPos = msg

    def run(self):

        while not rospy.is_shutdown():
            if self.enable:
                self.executeMission()
                #self._pubMsg(self.loiterPos, self.loiterPub)
                self.rate.sleep()

if __name__ == "__main__":
    mc = missionControl()
    mc.run()
