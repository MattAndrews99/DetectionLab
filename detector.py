#!/usr/bin/env python
"""
Process scan messages to detect intruders.

Subscribes to: /scan
Publishes to:  /intruder

Author: Nathan Sprague && 
Version: 

"""
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan

import numpy as np

class DetectorNode(rclpy.node.Node):

    def __init__(self):
        super().__init__('wander')

        self.prev_scan = None # Stores recently received scan messages.
        self.scan = None 

        self.intruder_pub = self.create_publisher(PointStamped, 'intruder', 10)
        self.create_timer(.1, self.timer_callback)

        self.create_subscription(LaserScan,'scan',
                                 self.scan_callback,
                                 qos_profile_sensor_data)

    def scan_callback(self, scan_msg):
        """Store the LaserScan msg."""
        self.prev_scan = self.scan
        self.scan = scan_msg
        self.angle_incr = scan_msg.angle_increment 

    def timer_callback(self):
        """Periodically check for intruders"""
        
        point = PointStamped()
        point.header.frame_id = "base_link"
        point.point.x = 1.0

        if self.prev_scan is not None:
            # TODO: PROCESS THE SCANS
            prev_arr = np.array(self.prev_scan.ranges)
            cur_arr = np.array(self.scan.ranges)
            prev_arr[np.isinf(prev_arr)] = 0
            prev_arr[np.isnan(prev_arr)] = 0
            cur_arr[np.isinf(cur_arr)] = 0
            cur_arr[np.isnan(cur_arr)] = 0
            changes = prev_arr - cur_arr
            absolute_changes = np.absolute(changes)
            #print(absolute_changes)
            detection = np.argwhere(absolute_changes > 0.1)
            if len(detection) > 0:
                print("DETECTED - Relative to Robot:")
                #print(detection)
                distance = cur_arr[detection[0]]
                angle = detection[0] * self.angle_incr
                y_comp = distance * np.sin(angle)
                x_comp = distance * np.cos(angle)
                print("\tx: {}".format(x_comp))
                print("\ty: {}".format(y_comp))
                point.point.x = float(x_comp)
                point.point.y = float(y_comp)
                self.intruder_pub.publish(point)

def main():
    rclpy.init()
    detector_node = DetectorNode()
    rclpy.spin(detector_node)

    thruster_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
