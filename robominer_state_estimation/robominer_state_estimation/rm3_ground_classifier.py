import rclpy
from rclpy.node import Node
from robominer_msgs.msg import SpectrometerMeasurement
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
import joblib
import os
import tf2_geometry_msgs
from std_msgs.msg import ColorRGBA
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Quaternion

# Get the current directory of the Python script
current_dir = os.path.dirname(os.path.abspath(__file__))

# Append the 'lib' directory to the module search path
lib_dir = os.path.join(current_dir, 'lib')


class FabricsClassifierNode(Node):
    def __init__(self):
        super().__init__('fabrics_classifier_node')
        self.model = joblib.load(lib_dir + "/fabrics_model.pkl")

        self.left_fluo_spectrum = 0
        self.left_refl_spectrum = 0
        self.left_fluo_received = False
        self.left_refl_received = False

        self.right_fluo_spectrum = 0
        self.right_refl_spectrum = 0
        self.right_fluo_received = False
        self.right_refl_received = False

        self.robot_pose = PoseStamped()
        self.robot_transform = TransformStamped()
        self.array_of_grounds = MarkerArray()
        self.marker_id = 0
        self.color_map = {
            "blue": (0.0, 0.0, 1.0),
            "red": (1.0, 0.0, 0.0),
            "green": (0.0, 1.0, 0.0),
            "beige": (0.96, 0.96, 0.86),
            "grass": (0.5, 0.2, 0.2)
        }

        self.body_to_left_spectro = TransformStamped()
        self.body_to_left_spectro.transform.translation.x = 0.24
        self.body_to_left_spectro.transform.translation.y = 0.32
        self.body_to_left_spectro.transform.rotation.x = 0.0
        self.body_to_left_spectro.transform.rotation = Quaternion(x=0.0,
                                                                  y=0.0,
                                                                  z=0.0,
                                                                  w=1.0)
        self.body_to_left_spectro.header.frame_id = "robot"
        self.body_to_left_spectro.child_frame_id = "left_spectro"

        self.body_to_right_spectro = TransformStamped()
        self.body_to_right_spectro.transform.translation.x = 0.24
        self.body_to_right_spectro.transform.translation.y = -0.32
        self.body_to_right_spectro.transform.rotation = Quaternion(x=0.0,
                                                                   y=0.0,
                                                                   z=0.0,
                                                                   w=1.0)
        self.body_to_left_spectro.header.frame_id = "robot"
        self.body_to_left_spectro.child_frame_id = "left_spectro"

        self.ohe = joblib.load(lib_dir + '/encoder.joblib')

        self.create_subscription(
            SpectrometerMeasurement,
            'spectrometer_left/reflectance_reading',
            self.left_refl_callback,
            10
        )

        self.create_subscription(
            SpectrometerMeasurement,
            'spectrometer_left/fluorescence_reading',
            self.left_fluo_callback,
            10
        )

        self.create_subscription(
            SpectrometerMeasurement,
            'spectrometer_right/reflectance_reading',
            self.right_refl_callback,
            10
        )

        self.create_subscription(
            SpectrometerMeasurement,
            'spectrometer_right/fluorescence_reading',
            self.right_fluo_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            'robot_pose_filtered',
            self.pose_callback,
            10
        )

        self.ground_array_publisher = self.create_publisher(
            MarkerArray, "marker_array_grounds_classified", 10)

    def process_spectrum(self, input_spectrum):
        input = np.array(input_spectrum)
        median_value = np.median(input)
        input = (input - median_value) / np.max(input)
        return list(input)

    def left_refl_callback(self, msg):
        self.left_refl_spectrum = self.process_spectrum(msg.spectrum)
        self.left_refl_received = True
        self.classify()

    def left_fluo_callback(self, msg):
        self.left_fluo_spectrum = self.process_spectrum(msg.spectrum)
        self.left_fluo_received = True
        self.classify()

    def right_refl_callback(self, msg):
        self.right_refl_spectrum = self.process_spectrum(msg.spectrum)
        self.right_refl_received = True
        self.classify()

    def right_fluo_callback(self, msg):
        self.right_fluo_spectrum = self.process_spectrum(msg.spectrum)
        self.right_fluo_received = True
        self.classify()

    def classify(self):
        if (
         self.left_fluo_received and
         self.left_refl_received and
         self.robot_pose._header._frame_id != ''):
            self.process_measurement(
                "left", self.left_fluo_spectrum, self.left_refl_spectrum)

        if (
         self.right_fluo_received and
         self.right_refl_received and
         self.robot_pose._header._frame_id != ''):
            self.process_measurement(
                "right", self.right_fluo_spectrum, self.right_refl_spectrum)

    def process_measurement(self, side, fluo_spectrum, refl_spectrum):
        concatenated_spectra = fluo_spectrum + refl_spectrum
        input_tensor = np.array(concatenated_spectra)
        input_tensor = input_tensor.reshape(1, -1)

        ground_pred = self.model.predict(input_tensor)
        prediction = np.zeros((1, 5))
        prediction[0][ground_pred] = 1
        label = self.ohe.inverse_transform(prediction)
        if side == "left":
            self.left_fluo_received, self.left_refl_received = False, False
            self.publish_marker(label[0][0], "left")
        if side == "right":
            self.right_fluo_received, self.right_refl_received = False, False
            self.publish_marker(label[0][0], "right")

        self.publish_prediction(label[0][0])
        # self.plot_spectra(input_tensor[0])

    def publish_marker(self, label, side):
        marker_location = self.get_sensor_position(side)
        marker_msg = Marker()
        marker_msg.header.frame_id = "odom"
        marker_msg.header.stamp = self.robot_pose.header.stamp
        marker_msg.id = self.marker_id
        self.marker_id += 1
        marker_msg.type = Marker.SPHERE
        marker_msg.action = Marker.ADD

        marker_msg.pose = marker_location

        marker_msg.scale.x = 0.1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1

        color = ColorRGBA()
        color.a = 1.0
        if label in self.color_map:
            color.r, color.g, color.b = self.color_map[label]
        else:
            color.r, color.g, color.b = 0, 0, 0

        marker_msg.color = color
        marker_msg.lifetime.sec = 0

        self.array_of_grounds.markers.append(marker_msg)
        self.ground_array_publisher.publish(self.array_of_grounds)

    def publish_prediction(self, label):
        Node.get_logger(self).info(f'The classifier recognized {label}')

    def plot_spectra(self, input_tensor):
        plt.plot(np.arange(576), input_tensor)
        plt.show()

    def get_sensor_position(self, side):
        pos = PoseStamped()
        if side == "left":
            tr = self.body_to_left_spectro
        elif side == "right":
            tr = self.body_to_right_spectro
        # Perform the transformation
        pos.pose.orientation = tr.transform.rotation
        pos.pose.position.x = tr.transform.translation.x
        pos.pose.position.y = tr.transform.translation.y
        pos.pose.position.z = tr.transform.translation.z

        pos.header = tr.header
        transformed_pose_stamped = tf2_geometry_msgs.do_transform_pose(
            pos.pose, self.robot_transform)
        return transformed_pose_stamped

        # except Exception as e:
        #     print('Failed to transform pose: {}'.format(str(e)))

    def pose_callback(self, msg):
        self.robot_transform.header = msg.header
        self.robot_transform.transform.rotation = msg.pose.orientation
        self.robot_transform.transform.translation.x = msg.pose.position.x
        self.robot_transform.transform.translation.y = msg.pose.position.y
        self.robot_transform.transform.translation.z = msg.pose.position.z
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose


def main(args=None):
    rclpy.init(args=args)
    fabrics_classifier_node = FabricsClassifierNode()
    rclpy.spin(fabrics_classifier_node)
    fabrics_classifier_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
