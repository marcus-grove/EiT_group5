#!/usr/bin/env python

import rospy

import mavros as mav
import mavros.utils
import mavros.command as mavCMD
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import (String, Int8, Float64, Bool)
mavros.set_namespace('mavros')

onB_StateSub = '/onboard/state'
mh_enableSub = '/onboard/enableMH'
commandSub   = '/gcs/command'
#isolatedSub  = '/onboard/isolated/llp_enable'

class droneCore():
    def __init__(self):
        ''' ROS node init '''
        rospy.init_node('DroneCoreNode')
        self.rate = rospy.Rate(20)
        
        ''' General variables ''' 
        self.MH_enabled = False
        self.sysState = 'idle' 
        self.MAVROS_State = mavros_msgs.msg.State()
        self.isAirbourne = False
        
        self.uavGPSPos = None
        self.uavLocalPos = mavSP.PoseStamped()
        self.uavHdg = None  

        ''' Subscribers '''
        # KillSwitch
        #rospy.Subscriber(isolatedSub, Bool, self._cb_onKillSwitch)

        # PX4 state 
        rospy.Subscriber(
            mavros.get_topic('state'),
            mavros_msgs.msg.State,
            self._cb_uavState) # 
        
        # PX4 global position (GPS)
        rospy.Subscriber(
            mavros.get_topic('global_position','global'),
            NavSatFix, 
            self._cb_SatFix)

        # PX4 global compass heading (GPS)
        rospy.Subscriber(
            mavros.get_topic('global_position', 'compass_hdg'),
            Float64, 
            self._cb_headingUpdate)

        # Ground control commands
        rospy.Subscriber(
            commandSub,
            Int8,
            self._cb_onCommand)

        ''' Publishers '''
        # Onboard state
        self.statePub = rospy.Publisher(
            onB_StateSub, 
            String,
            queue_size=1)

        # Enable message handler
        self.enableMHPub = rospy.Publisher(
            mh_enableSub,
            Bool,
            queue_size=1
        )
        
        #self.llpPub = rospy.Publisher(isolatedSub, Bool, queue_size=1)
        self.spLocalPub = mavSP.get_pub_position_local(queue_size=5)

        ''' Services '''
        self.setMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        self.enableTakeoff = rospy.ServiceProxy('/ mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)

        # Perform MAVROS handshake   
        self._mavrosHandshake()

    ''' Core Functions '''
    def _mavrosHandshake(self): 
        rospy.loginfo('DroneCore: Waiting for MAVROS Connection.')
        i=0
        time = rospy.Time.now()
        for i in range(0,3):
            print'.',
            if self.MAVROS_State.connected:
                rospy.loginfo("DroneCore: MAVROS Connected!")
                break
            rospy.sleep(1)
        if not self.MAVROS_State.connected:
            errorMsg = "DroneCore: MAVROS not connected!"
            rospy.logfatal(errorMsg)
            rospy.signal_shutdown(errorMsg)
    
    # Generic function to publish a message
    def _pubMsg(self, msg, topic):
        msg.header = mavSP.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    # Fuction for updating onboard state
    def _setState(self, state):
        self.sysState = state
        if state == 'idle':
            self.enableMHPub.publish(False)
        elif not self.MH_enabled:
            self.enableMHPub.publish(True)
            self.MH_enabled = True
        self.statePub.publish(state)
        rospy.loginfo('DroneCore: state = {}'.format(state))

    ''' Subscriber callbacks '''
    #def _cb_onKillSwitch(self,msg):
        #if msg.data==True:
            #self.enableMHPub.publish(False)
            #self.MH_enabled = False
            #self.statePub.publish('isolate')

    def _cb_onCommand(self, msg):
        command = str(chr(msg.data))
        command.lower()
    
        if command == 't': # Takeoff
            self.droneTakeoff()
            if self.sysState == 'takeoff':
                self._setState('loiter')
        if command == 'o': # Offboard control
            if self.sysState == 'idle':
                self.enableMHPub.publish(True) # send messages to enable offboard
                self._setState('loiter')
                for i in range(0,3):
                    resp = self.setMode(0,'OFFBOARD')
                    if self.MAVROS_State.mode == 'OFFBOARD':
                        rospy.loginfo('DroneCore: PX4 mode = OFFBOARD')
                        break
            else:
                self.enableMHPub.publish(False)
                self._setState('idle')
                for i in range(0,3):
                    resp = self.setMode(0,'AUTO.LOITER')
                    if self.MAVROS_State.mode == 'AUTO.LOITER':
                        rospy.loginfo('DroneCore: PX4 mode = AUTO.LOITER')
                        break
        if command == 'h': # Returns the drone to home
            pass
        if command == 'v': # Perform vision guided landing
            self._setState('vision_land')
        if command == 'm': # Execute mission
            if self.MAVROS_State.mode != 'OFFBOARD':
                rospy.logwarn('DroneCore: OFFBOARD not enabled')
            else:
                self._setState('mission')
        if command == 'k': # Kill switch 
            self.enableMHPub.publish(False)
            self._setState('idle')
            #TODO: Implement PX4 kill switch
        if command == 'r': # reset ROS framwork
            pass

    def _cb_SatFix(self, msg):
        self.uavGPSPos = msg
    
    def _cb_headingUpdate(self,msg):
        self.uavHdg = msg
    
    def _cb_uavState(self, msg):
        self.MAVROS_State = msg
        if self.sysState != 'idle' and self.MAVROS_State.mode != 'OFFBOARD':
             rospy.logwarn("DroneCore: System enabled, but drone is in manual control. Disabling Message Handler")
             self._setState('idle')
        pass

    ''' Functions '''
    def droneTakeoff(self, alt=1.0):
        if self.isAirbourne == False or self.sysState == 'idle':
            if not self.MAVROS_State.armed:
                mavCMD.arming(True)
                rospy.loginfo('DroneCore: Arming')

            preArmMsgs = self.uavLocalPos
            preArmMsgs.pose.position.z = 1.5
            rospy.loginfo('DroneCore: Takeoff altitude = {} m'.format(preArmMsgs.pose.position.z))

            for i in range(50):
                self._pubMsg(preArmMsgs, self.spLocalPub)
                self.rate.sleep()

            self._setState('takeoff')
            self.setMode(0, 'OFFBOARD')
            self.isAirbourne = True
            rospy.loginfo('DroneCore: UAV is airbourne')
            rospy.loginfo('DroneCore: PX4 mode = OFFBOARD')
            self.enableMHPub.publish(self.isAirbourne)
            #wait until takeoff has occurred
            while(self.uavLocalPos.pose.position.z <= (preArmMsgs.pose.position.z-0.25)):
                self._pubMsg(preArmMsgs, self.spLocalPub)

            rospy.loginfo('DroneCore: Takeoff complete')

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
        pass

if __name__ == "__main__":
    dc = droneCore()
    dc.run()
