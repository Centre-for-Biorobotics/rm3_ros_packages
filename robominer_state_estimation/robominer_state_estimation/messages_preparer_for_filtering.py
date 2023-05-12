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

Variances for speed estimation using the dynamics model (Vvx, Vvy .....Vwz),
0.01    0.0016         0         0         0    0.0324
=> we take (0.01  0.002  0  0  0  0.033)

For bno_IMU:
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
            '/bno080_imu',
            self.bn0_imu_received,
            10)
        self.subscription_1  # prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            TwistStamped,
            '/twist_FKinematics_stamped',
            self.twist_received_kinematics,
            10)
        self.subscription_2  # prevent unused variable warning

        self.subscription_3 = self.create_subscription(
            Imu,
            '/pi48_imu/data_raw',
            self.pi48_imu_received,
            10)
        self.subscription_3  # prevent unused variable warning

        self.subscription_4 = self.create_subscription(
            PoseStamped,
            '/robot_pose_filtered',
            self.pose_received,
            10)
        self.subscription_4  # prevent unused variable warning

        self.subscription_5 = self.create_subscription(
            Odometry,
            '/estimated_odom_FDynamics',
            self.odom_received_dynamics,
            10)
        self.subscription_5  # prevent unused variable warning

        self.subscription_6 = self.create_subscription(
            Imu,
            '/camera_jetson/imu',
            self.camera_jetson_imu_received,
            10)
        self.subscription_6  # prevent unused variable warning

        # Create publisher(s)

        self.publisher_bno_imu = self.create_publisher(
            Imu,
            '/bno080_imu_with_cov',
            10)
        self.publisher_pi48 = self.create_publisher(
            Imu,
            '/pi48_imu_with_cov',
            10)
        self.publisher_camera_jetson_imu = self.create_publisher(
            Imu,
            '/camera_jetson_imu_with_cov',
            10)
        self.publisher_twist_kinematics = self.create_publisher(
            TwistWithCovarianceStamped,
            '/twist_with_cov_kinematics',
            10)
        # self.publisher_twist_dynamics = self.create_publisher(
        #     TwistWithCovarianceStamped,
        #     '/twist_with_cov_dynamics',
        #     10)
        self.publisher_odom_dynamics = self.create_publisher(
            Odometry,
            '/odom_with_cov_dynamics',
            10)

        self.publisher_pose = self.create_publisher(
            Odometry,
            '/robot_pose_gt_initial_pose_frame',
            10)

    def bn0_imu_received(self, msg):
        msg.header.frame_id = "bno_imu_frame"
        msg.orientation_covariance = [1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0]
        msg.angular_velocity_covariance = [0.0027, 0.0, 0.0,
                                           0.0, 0.0027, 0.0,
                                           0.0, 0.0, 0.0027]
        msg.linear_acceleration_covariance = [0.09, 0.0, 0.0,
                                              0.0, 0.09, 0.0,
                                              0.0, 0.0, 0.09]
        self.publisher_bno_imu.publish(msg)

    def pi48_imu_received(self, msg):
        msg.header.frame_id = "pi48_imu_frame"
        msg.orientation_covariance = [1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0]
        msg.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                           0.0, 0.001, 0.0,
                                           0.0, 0.0, 0.001]
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                              0.0, 0.01, 0.0,
                                              0.0, 0.0, 0.01]
        self.publisher_pi48.publish(msg)

    def camera_jetson_imu_received(self, msg):
        msg.header.frame_id = "camera_jetson_accel_frame"
        msg.orientation_covariance = [1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0]
        msg.angular_velocity_covariance = [0.0027, 0.0, 0.0,
                                           0.0, 0.0027, 0.0,
                                           0.0, 0.0, 0.0027]
        msg.linear_acceleration_covariance = [0.09, 0.0, 0.0,
                                              0.0, 0.09, 0.0,
                                              0.0, 0.0, 0.09]
        self.publisher_camera_jetson_imu.publish(msg)

    def twist_received_kinematics(self, msg):
        new_msg = TwistWithCovarianceStamped()
        new_msg.header = msg.header
        new_msg.header.frame_id = "base_link_est_ukf"
        new_msg.twist.twist = msg.twist
        slip_ratio = 0.30
        new_msg.twist.twist.linear.x *= (1 - slip_ratio)
        new_msg.twist.twist.linear.y *= (1 - slip_ratio)
        new_msg.twist.twist.angular.z *= (1 - slip_ratio)
        new_msg.twist.covariance = [0.025, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.005, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
                                    0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, pow(10, -9), 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.05]
        self.publisher_twist_kinematics.publish(new_msg)

    # def odom_received_dynamics(self, msg):
    #     new_msg = TwistWithCovarianceStamped()
    #     new_msg.header = msg.header
    #     new_msg.header.frame_id = "base_link_est_ukf"
    #     new_msg.twist.twist = msg.twist.twist
    #     slip_ratio = 0.30
    #     new_msg.pose.pose.position.x *= (1 - slip_ratio)
    #     new_msg.pose.pose.position.y *= (1 - slip_ratio)
        
    #     new_msg.twist.twist.linear.x *= (1 - slip_ratio)
    #     new_msg.twist.twist.linear.y *= (1 - slip_ratio)
    #     new_msg.twist.twist.angular.z *= (1 - slip_ratio)
    #     new_msg.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
    #                                 0.0, 0.002, 0.0, 0.0, 0.0, 0.0,
    #                                 0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
    #                                 0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
    #                                 0.0, 0.0, 0.0, 0.0, pow(10, -9), 0.0,
    #                                 0.0, 0.0, 0.0, 0.0, 0.0, 0.033]
    #     self.publisher_twist_dynamics.publish(new_msg)

    def odom_received_dynamics(self, msg):
        new_msg = msg
        new_msg.header.frame_id = "odom"
        slip_ratio = 0.30
        new_msg.twist.twist.linear.x *= (1 - slip_ratio)
        new_msg.twist.twist.linear.y *= (1 - slip_ratio)
        new_msg.twist.twist.angular.z *= (1 - slip_ratio)
        new_msg.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.002, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
                                    0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, pow(10, -9), 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.033]
        new_msg.twist.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.002, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
                                    0.0, 0.0, 0.0, pow(10, -9), 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, pow(10, -9), 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.033]
        self.publisher_odom_dynamics.publish(new_msg)

    def pose_received(self, msg):
        new_msg = Odometry()
        new_msg.header = msg.header
        new_msg.header.frame_id = "gt_initial_pose"
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
