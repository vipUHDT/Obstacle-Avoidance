#!/usr/bin/env python

import rospy
import _thread
import threading
import time
import mavros

from math import *
from mavros.utils import *
from mavros_msgs.msg import State, ParamValue, GlobalPositionTarget
from mavros_msgs.srv import ParamSet
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from haversine import haversine, Unit


class SetpointGlobalPosition:
    """
    This class sends position targets to FCU's position controller
    """
    def __init__(self):
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_alt = 0.0

        self.done = False
        self.current_state = State()


        self.pub = rospy.Publisher(
            '/mavros/setpoint_position/global',
            GeoPoseStamped,
            queue_size=10
        )
        
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.position_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)

        threading.Thread(target=self.navigate).start()

        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()
        
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def position_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.current_alt = msg.altitude
        #rospy.loginfo(f"Position Updated: {self.current_lat}, {self.current_lon}, {self.current_alt}")

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


    def navigate(self):
        rate = rospy.Rate(10)   # 10hz
        msg = GeoPoseStamped() #Change this back

        '''
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
        '''
        while not rospy.is_shutdown():
            msg.pose.position.latitude = self.latitude
            msg.pose.position.longitude = self.longitude
            msg.pose.position.altitude = self.altitude

            self.pub.publish(msg)
            rate.sleep()

    def set(self, latitude, longitude, altitude, delay=0, wait=True):
        self.done = False
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

        time.sleep(delay)



    def reached(self, current_lat, current_lon, current_alt):
        def is_near(value, target, threshold):
            return abs(value - target) < threshold
        '''
        if is_near(curent_lat, self.latitude, 25) and \
           is_near(current_lon, self.longitude,25) and \
           is_near(current_alt, self.altitude,25):
            self.done = True
            self.done_evt.set()
            return True
        '''
        distance = haversine((self.current_lat, self.current_lon), (self.latitude, self.longitude), unit=Unit.METERS)
        if distance < 3 :
            return True
            
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

    waypoints = [
        (21.4004242, -157.7647783, 10),
        (21.4006577, -157.7642378, 10)
    ]

    for waypoint in waypoints:
        latitude, longitude, altitude = waypoint
        rospy.loginfo(f"Navigating to waypoint:")
        setpoint.set(latitude, longitude, altitude)

        while not rospy.is_shutdown():
            if setpoint.reached(setpoint.current_lat, setpoint.current_lon, setpoint.current_alt):
                rospy.loginfo(f"Waypoint reached:")
                break
            rate.sleep()


    rospy.loginfo("Bye!")

if __name__ == '__main__':
    setpoint_demo()

