#!/usr/bin/env python

import rospy
import _thread
import threading
import time
import mavros

from math import *
from mavros.utils import *
from mavros import setpoint as SP
from mavros_msgs.msg import State, ParamValue, GlobalPositionTarget
from mavros_msgs.srv import ParamSet
from geographic_msgs.msg import GeoPoseStamped


class SetpointGlobalPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.done = False
        self.current_state = State()


        self.pub = rospy.Publisher(
            '/mavros/setpoint_position/global',
            GlobalPositionTarget,
            queue_size=10
        )
        

        state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)

        threading.Thread(target=self.navigate).start()

        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()
        
        
    def state_callback(self, msg):
        self.current_state = msg
        

    def is_armed(self):


        rospy.loginfo("Waiting to be Armed...")
        while not rospy.is_shutdown():
            if self.current_state and self.current_state.armed:
                rospy.loginfo("ARMED")
                break
            rospy.sleep(1)

    def is_guided(self):
        rospy.loginfo("Checking drone status")
        while not rospy.is_shutdown():
            if (self.current_state and self.current_state.guided) and (self.current_state and self.current_state.armed):
                rospy.loginfo("GUIDED")
                break
            rospy.sleep(1)
        
    def set_speed(self, spd):
        self.speed = spd


    def navigate(self):
        rate = rospy.Rate(10)   # 10hz
        msg = GlobalPositionTarget

        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX |
            GlobalPositionTarget.IGNORE_VY |
            GlobalPositionTarget.IGNORE_VZ |
            GlobalPositionTarget.IGNORE_AFX |
            GlobalPositionTarget.IGNORE_AFY |
            GlobalPositionTarget.IGNORE_AFZ |
            GlobalPositionTarget.IGNORE_YAW |
            GlobalPositionTarget.IGNORE_YAW_RATE
        )

        while not rospy.is_shutdown():
            msg.latitude = self.latitude
            msg.longitude = self.longitude
            msg.altitude = self.altitude

            self.pub.publish(msg)
            rate.sleep()

    def set(self, latitude, longitude, altitude, delay=0, wait=True):
        self.done = False
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

        if wait:
            rate = rospy.Rate(5)
            while not self.done and not rospy.is_shutdown():
                rate.sleep()

        time.sleep(delay)

    def reached(self, curent_lat, current_lon, current_alt):
        def is_near(value, target, threshold):
            return abs(value - target) < threshold

        if is_near(curent_lat, self.latitude, 0.5) and \
           is_near(current_lon, self.longitude) and \
           is_near(current_alt, self.altitude, self.z):
            self.done = True
            self.done_evt.set()

def set_param(param_name, param_value):
        rospy.wait_for_service('/mavros/param/set')
        set_param_service = rospy.ServiceProxy('/mavros/param/set', ParamSet)

        param = ParamValue()
        param.integer = int(param_value)
        param.real = float(param_value)

        response = set_param_service(param_name, param)
        if response.success:
            rospy.loginfo(f"Sucessfully set Parameter {param_name} to {param_value}")
        else:
            rospy.loginfo(f"Failed to set parameter {param_name}")

def setpoint_demo():
    rospy.init_node('auto_test1')
    mavros.set_namespace()  # initialize mavros module with default namespace
    rate = rospy.Rate(10)

    setpoint = SetpointGlobalPosition()
    set_param("WPNAV_SPEED", 2000)
    rospy.loginfo("Init")

    setpoint.is_armed()
    setpoint.is_guided()

    setpoint.set_speed(40)

    #rospy.loginfo("Climb")
    #setpoint.set(0.0, 0.0, 3.0, 0)
    #setpoint.set(0.0, 0.0, 10.0, 5)

    rospy.loginfo("Sink")

    rospy.loginfo("Fly to the right")
    setpoint.set(21.4004242, -157.7647783, 30, 5)

    rospy.loginfo("Fly to the left")
    setpoint.set(21.4006577, -157.7642378, 30, 5)


    rospy.loginfo("Bye!")

if __name__ == '__main__':
    setpoint_demo()

