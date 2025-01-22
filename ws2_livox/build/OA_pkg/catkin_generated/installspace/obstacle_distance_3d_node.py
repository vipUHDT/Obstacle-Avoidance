#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from pymavlink import mavutil, mavlink
import math

class ObstacleDistance3D:
    def __init__(self):
        # Set up MAVLink connection to ArduPilot
        self.mavlink_connection = mavutil.mavlink_connection('udp:127.0.0.1:14551', baud = 57600)
        self.mavlink_connection.wait_heartbeat()
        print( "Heartbeat from sustem {system %u component %u }" %( self.mavlink_connection.target_system, self.mavlink_connection.target_component ) )

        # Subscribe to the downsampled point cloud topic
        rospy.Subscriber("/voxels/output", PointCloud2, self.pointcloud_callback)

    def pointcloud_callback(self, msg):
        # Process the downsampled point cloud
        obstacle_data = []

        # Iterate over points in the downsampled cloud
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point  # Extract 3D coordinates
          

            # Only consider obstacles within a certain range
            distance = (x**2 + y**2 + z**2) ** 0.5
            if 1 < distance < 30:  # Only consider obstacles between 1m and 30m
                obstacle_data.append((x, y, z))

        # Send the obstacle data as OBSTACLE_DISTANCE_3D messages
        for distance, azimuth, elevation in obstacle_data:
            # self.send_obstacle_distance_3d(distance, azimuth, elevation)
            self.send_obstacle_distance_3d(x, y, z)

    def send_obstacle_distance_3d(self, x, y, z):
        """
        Send an OBSTACLE_DISTANCE_3D MAVLink message to ArduPilot.
        """
        timestamp = int(rospy.get_time() * 1e6)  # Timestamp in microseconds
        frame = mavlink.MAV_FRAME_BODY_NED

        # Create and send the MAVLink message
        self.mavlink_connection.mav.obstacle_distance_3d_send(
            timestamp,
            0,
            frame,
            65535,
            x,      # Obstacle distance in meters
            y,       # Azimuth angle in degrees
            z,     # Elevation angle in degrees
            0,             # Optional: Minimum detectable obstacle distance
            0
        )
        print("OBSTACLE_DISTANCE_3D message X: {}, Y: {}, Z: {}".format(x, y, z))

if __name__ == '__main__':
    rospy.init_node('obstacle_distance_3d_node')
    obstacle_distance_node = ObstacleDistance3D()
    rospy.spin()


