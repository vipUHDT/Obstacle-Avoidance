#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix

def gps_callback(msg):
    rospy.loginfo(f"Latitude: {msg.latitude:.7f}, Longitude: {msg.longitude:.7f}, Altitude: {msg.altitude:.2f} m")
    rospy.signal_shutdown("Got GPS data, exiting.")  # Shutdown the node after first message

if __name__ == '__main__':
    rospy.init_node('collect_gps')
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, gps_callback)
    rospy.loginfo("Waiting for GPS")
    rospy.spin()