#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from pointcloud_to_laserscan import pointcloud_to_laserscan

def pointcloud_callback(msg):
    # Create a LaserScan message
    laser_scan = LaserScan()

    # Use the pointcloud_to_laserscan method to convert PointCloud2 to LaserScan
    try:
        # Here you can set your conversion parameters, like the sensor frame and ranges
        laser_scan = pointcloud_to_laserscan.convert(msg)

        # Set header for LaserScan (important for correct timestamp and frame_id)
        laser_scan.header.stamp = rospy.Time.now()
        laser_scan.header.frame_id = "livox_frame"  # Make sure this matches your LiDAR frame

        # Publish the LaserScan message
        laser_scan_pub.publish(laser_scan)
    except Exception as e:
        rospy.logerr(f"Error converting point cloud to LaserScan: {e}")

def main():
    # Initialize the ROS node
    rospy.init_node('livox_to_laserscan_node')

    # Subscribe to the Livox LiDAR PointCloud2 data
    rospy.Subscriber('/livox/point_cloud', PointCloud2, pointcloud_callback)

    # Publish converted LaserScan messages
    global laser_scan_pub
    laser_scan_pub = rospy.Publisher('/livox/scan', LaserScan, queue_size=10)

    # Spin to keep the node running
    rospy.spin()

if __name__ == "__main__":
    main()


