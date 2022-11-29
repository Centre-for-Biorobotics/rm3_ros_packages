''' ####################
    This node prepares the data to be used by the kalman filter.
    It takes the IMU and body velocity messages and adds covariance
    matrices to them before republishing them, and multiplies the twist
    message values. It also takes the pose received and converts it to
    an odometry message.
    ==================================
    Author: Simon Godon
    Date: June 10, 2022

Variances for speed estimation using kinematics model (Vvx, Vvy .....Vwz),
 computed for the first 4 runs =
0.0146    0.0028         0         0         0    0.0397
0.0158    0.0036         0         0         0    0.0477
0.0221    0.0043         0         0         0    0.0410
0.0225    0.0039         0         0         0    0.0443
=> we take (0.025  0.005  0  0  0  0.05)


For IMU:
Accelerometer Accuracy: 0.3m/s2 => variance = 0.09m2/s4
Gyroscope Accuracy: 3.1°/sec = 0.052 rad/sec => variance = 0.0027 rad/sec2

    #################### '''

import rclpy
from tf2_ros import TransformBroadcaster
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class PublishingSubscriber(Node):
    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('message_formatter_node')
        self.tf_broadcaster = TransformBroadcaster(self)
        # Create subscriber(s)
        self.subscription_1 = self.create_subscription(
            Imu,
            '/front_imu',
            self.imu_received,
            10)
        self.subscription_1  # prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            TwistStamped,
            '/cmd_vel_stamped',
            self.twist_received,
            10)
        self.subscription_2  # prevent unused variable warning

        self.subscription_3 = self.create_subscription(
            Imu,
            '/imu',
            self.front_imu_received,
            10)
        self.subscription_3  # prevent unused variable warning

        self.subscription_4 = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_received,
            10)
        self.subscription_4  # prevent unused variable warning

        # Create publisher(s)

        self.publisher_imu = self.create_publisher(
            Imu,
            '/imu_with_cov',
            10)
        self.publisher_front_imu = self.create_publisher(
            Imu,
            'front_imu_with_cov',
            10)
        self.publisher_twist = self.create_publisher(
            TwistWithCovarianceStamped,
            '/twist_with_cov',
            10)

        self.publisher_pose = self.create_publisher(
            Odometry,
            '/robot_pose_world_frame',
            10)

    def imu_received(self, msg):
        msg.header.frame_id = "imu_frame"
        msg.orientation_covariance = [1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0]
        msg.angular_velocity_covariance = [0.0027, 0.0, 0.0,
                                           0.0, 0.0027, 0.0,
                                           0.0, 0.0, 0.0027]
        msg.linear_acceleration_covariance = [0.09, 0.0, 0.0,
                                              0.0, 0.09, 0.0,
                                              0.0, 0.0, 0.09]
        self.publisher_imu.publish(msg)

    def front_imu_received(self, msg):
        msg.header.frame_id = "front_imu_frame"
        msg.orientation_covariance = [1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0]
        msg.angular_velocity_covariance = [0.0027, 0.0, 0.0,
                                           0.0, 0.0027, 0.0,
                                           0.0, 0.0, 0.0027]
        msg.linear_acceleration_covariance = [0.09, 0.0, 0.0,
                                              0.0, 0.09, 0.0,
                                              0.0, 0.0, 0.09]
        self.publisher_front_imu.publish(msg)

    def twist_received(self, msg):
        new_msg = TwistWithCovarianceStamped()
        new_msg.header = msg.header
        new_msg.twist.twist = msg.twist
        new_msg.twist.twist.linear.x *= 0.35
        new_msg.twist.twist.linear.y *= 0.35
        new_msg.twist.covariance = [0.025, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.005, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
                                    0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, pow(10, -9), 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
        self.publisher_twist.publish(new_msg)

    def pose_received(self, msg):
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.header.frame_id = "world"
        new_msg.child_frame_id = "robot_ground_truth"
        new_msg.pose.pose = msg.pose
        self.publisher_pose.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    publishing_subscriber = PublishingSubscriber()
    rclpy.spin(publishing_subscriber)
    publishing_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
