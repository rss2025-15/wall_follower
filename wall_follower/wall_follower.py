#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
import time

from std_msgs.msg import Header
from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        
        

        # Fetch constants from the ROS parameter server
        # This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.parameters_callback)


		
	# TODO: Initialize your publishers and subscribers here
        self.subscription_=self.create_subscription(LaserScan, self.SCAN_TOPIC, self.callback, 1)
        self.publisher_=self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 1)
        self.line_pub = self.create_publisher(Marker, "laser", 1)

        self.point_viz = self.create_publisher(LaserScan, "point_viz", 1)

    # error increment
        self.start = time.time()
        self.error_pre = 0
        self.i_ctrl = 0

        self.clock = self.get_clock()
        self.start = self.clock.now()
        self.time_prev = self.clock.now()

        #WHY DOESNT THSI UPDATE ITSELF MAN
        self.SIDE = 1
        self.DESIRED_DISTANCE = 1.

        self.counter = 0



    # TODO: Write your callback functions here 
    def callback(self, lidar_msg):

        Kp = 1.
        Kd = .1
        Ki = .1

        
        angles = np.linspace(lidar_msg.angle_min, lidar_msg.angle_max, len(lidar_msg.ranges))
        # self.get_logger().info("'%s'" % self.SIDE)
        if self.SIDE>0:
            slice_start, slice_stop = int(((0.05)*np.pi - lidar_msg.angle_min)/lidar_msg.angle_increment), int(((0.4)*np.pi - lidar_msg.angle_min)/lidar_msg.angle_increment)
        else:
            slice_start, slice_stop = int((-(0.4)*np.pi - lidar_msg.angle_min)/lidar_msg.angle_increment), int((-(0.05)*np.pi - lidar_msg.angle_min)/lidar_msg.angle_increment)
        
        sliced_angles = angles[slice_start:slice_stop]
        sliced_lidar = np.array(lidar_msg.ranges[slice_start:slice_stop])
        close = np.where(sliced_lidar <2.0)
        sliced_angles = sliced_angles[close]
        sliced_lidar = sliced_lidar[close]


        lidar_x = np.cos(sliced_angles, dtype = 'float')*sliced_lidar
        lidar_y = np.sin(sliced_angles, dtype = 'float')*sliced_lidar
        A = np.vstack([lidar_x, np.ones(len(lidar_x))]).T
        m, c = np.linalg.lstsq(A, lidar_y)[0]
        # self.get_logger().info("m, c '%s' '%s'" % (m, c))
        
        
        if sliced_angles.any():

            lsr_x = np.linspace(np.min(lidar_x), np.max(lidar_x), len(lidar_x))
            lsr_lidar = m * lsr_x + c
            VisualizationTools.plot_line(lsr_x, lsr_lidar, self.line_pub, frame="/laser")

            used_points_msg = lidar_msg
            used_points_msg.angle_min = sliced_angles.min()
            used_points_msg.angle_max = sliced_angles.max()
            used_points_msg.ranges = sliced_lidar.tolist()

            self.point_viz.publish(used_points_msg)

            error = (-1)*self.SIDE*self.DESIRED_DISTANCE - (-1)*self.SIDE*np.min(np.sqrt(np.square(lsr_x) + np.square(lsr_lidar))) 
            dt = self.clock.now() - self.time_prev
            t = self.clock.now() - self.start

            # check for stuff right in front of car
            alpha = 1.0
            if np.min(np.abs(sliced_angles)) < 0.16*np.pi:
                error += alpha*(-1)*self.SIDE*(1/np.min(np.sqrt(np.square(lsr_x) + np.square(lsr_lidar))))
           

            p_ctrl = Kp*error
            d_ctrl = Kd*(error - self.error_pre)/(dt.nanoseconds/1e9)
            self.i_ctrl += Ki*error*(dt.nanoseconds/1e9)/(t.nanoseconds/1e9)

            # ctrl_output = p_ctrl+d_ctrl+self.i_ctrl
            ctrl_output = p_ctrl +d_ctrl
            self.error_pre = error
            self.get_logger().info("error '%s'" % ctrl_output)
        
        
        else:
           
            ctrl_output = 2.0*self.SIDE
            self.get_logger().info("error '%s'" % ctrl_output)
            # ctrl_output=0.0        
        



        drive_msg = AckermannDriveStamped()
        # drive_msg.header = Header()
        # drive_msg.header.frame_id = 

        drive_msg.drive.steering_angle = ctrl_output
        drive_msg.drive.steering_angle_velocity = 0.0
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.acceleration = 0.0
        drive_msg.drive.jerk = 0.0

        self.publisher_.publish(drive_msg)
        # self.get_logger().info("Slices  '%s' '%s'" % (slice_start, slice_stop))
    
    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    