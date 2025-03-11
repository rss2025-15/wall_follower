#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", 1)
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 1.0)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value
		
        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)
  
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 1)
        self.scan_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.scan_callback, 10)
        self.marker_pub = self.create_publisher(Marker, "/wall", 1)

        self.prev_error = 0
        self.prev_time = self.get_clock().now().nanoseconds * 1e-9

        self.Kp = 1.2
        self.Kd = 0.6
        self.total = 0
        self.count = 0
        self.abs_e = 0
        self.lidar_error =0
    
    def scan_callback(self, scan):
        # find angles in range
        ranges = np.array(scan.ranges)
        angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

        if self.SIDE == 1: # left side
            lower_bound = np.deg2rad(60)
            upper_bound = np.deg2rad(120)
        else: # right side
            lower_bound = np.deg2rad(-120)
            upper_bound = np.deg2rad(-60)

        target_angle = np.deg2rad(90 * self.SIDE)

        # find the indices of the angles that are within the bounds
        indices = np.where((angles >= lower_bound) & (angles <= upper_bound))
        filtered_ranges = ranges[indices]
        filtered_angles = angles[indices]

        if len(filtered_ranges) == 0:
            self.publish_drive(0)
            return

        x = filtered_ranges * np.cos(filtered_angles)
        y = filtered_ranges * np.sin(filtered_angles)

        A = np.vstack([x, np.ones(len(x))]).T
        m, b = np.linalg.lstsq(A, y, rcond=None)[0]

        distance = abs(b) / np.sqrt(1 + m**2)
        self.get_logger().info(f"Distance: {distance}, Side: {self.SIDE}")

        error = (distance - self.DESIRED_DISTANCE) * self.SIDE
        current_time = self.get_clock().now().nanoseconds * 1e-9
        dt = current_time - self.prev_time if current_time - self.prev_time > 0 else 0.001
        d_error = (error - self.prev_error) / dt

        front_lower_bound = np.deg2rad(-0.16*np.pi)
        front_upper_bound = np.deg2rad(0.16*np.pi)
        front_indices = np.where((angles >= front_lower_bound) & (angles <= front_upper_bound))
        if np.any((np.array(filtered_ranges)) <self.DESIRED_DISTANCE+0.5):
            
            alpha = 1.5
            front_angles = np.where(ranges[front_indices]<self.DESIRED_DISTANCE+0.5)
            
            if np.any(front_angles):
                self.get_logger().info(f"{np.min(np.abs(angles[front_angles]))}")
                error += alpha*self.SIDE*1/distance
            steering_angle = self.Kp * error + self.Kd * d_error
        else:
            self.get_logger().info(f"cannot see!!{self.count}")
            steering_angle = 2.0*self.SIDE 
        self.publish_drive(steering_angle)
        
        self.prev_error = error
        self.prev_time = current_time

        self.publish_wall_marker(scan, m, b, x, y)
        self.total += error**2
        self.abs_e += abs(error)
        start, stop = int(((0.45)*np.pi - scan.angle_min)/scan.angle_increment), int(((0.55)*np.pi - scan.angle_min)/scan.angle_increment)
        self.get_logger().info(f"{start} {stop} {str(scan.ranges[start:start+1])}")
        self.lidar_error+=abs(self.DESIRED_DISTANCE-np.mean(np.array(scan.ranges[start:stop])))
        self.count += 1

        # self.get_logger().info(f"total: {self.total}, count: {self.count}")
        self.get_logger().info(f"mse: {(self.total/self.count)**0.5}, abs e: {self.abs_e/self.count} lidar errror:{self.lidar_error/self.count} count: {self.count}")

    def publish_drive(self, steering_angle):
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    def publish_wall_marker(self, scan, m, b, x_points, y_points):
        marker = Marker()
        marker.header.frame_id = scan.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "wall"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        x_min, x_max = min(x_points), max(x_points)

        num_line_points = 10
        xs = np.linspace(x_min, x_max, num_line_points)
        for xi in xs:
            yi = m * xi + b
            marker.points.append(self.create_point_msg(xi, yi, 0.0))

        self.marker_pub.publish(marker)

        points_marker = Marker()
        points_marker.header.frame_id = scan.header.frame_id
        points_marker.header.stamp = self.get_clock().now().to_msg()
        points_marker.ns = "wall_points"
        points_marker.id = 1
        points_marker.type = Marker.POINTS
        points_marker.action = Marker.ADD

        points_marker.scale.x = 0.03
        points_marker.scale.y = 0.03
        points_marker.color.a = 1.0
        points_marker.color.r = 0.0
        points_marker.color.g = 1.0
        points_marker.color.b = 0.0

        for xi, yi in zip(x_points, y_points):
            points_marker.points.append(self.create_point_msg(xi, yi, 0.0))

        self.marker_pub.publish(points_marker)

    def create_point_msg(self, x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point
    
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
    