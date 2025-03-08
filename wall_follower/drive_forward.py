import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


class DriveForward(Node):

    def __init__(self):
        super().__init__('drive_forward')

        # HIGH LEVEL TOPICS
        # topic_name = "/vesc/high_level/input/nav_0"
        # topic_name = "/vesc/high_level/input/nav_1"
        topic_name = "/vesc/high_level/input/nav_2"

        # LOW LEVEL SAFETY TOPIC
        # topic_name = "/vesc/low_level/input/safety"
        print(f"Using topic {topic_name}")
        self.publisher_ = self.create_publisher(AckermannDriveStamped, topic_name, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        """
        Callback function.
        """
        msg = AckermannDriveStamped()

        # Go in a circle
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = 0.
        msg.drive.speed = 2.
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = DriveForward()

    rclpy.spin(node)

    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
