#!/usr/bin/python3

"""Node that uses opencv to detect aruco tags and publish their pose.

@author: Roza Gkliva
@contact: roza.gkliva@ttu.ee
@date: 25-08-2021
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import transforms3d

import tf2_ros

import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
#from robominer_msgs.msg import FiducialTransformArray, FiducialTransform

from geometry_msgs.msg import TransformStamped

from icecream import ic

ic.enable()

class ArucoDetect(Node):
    """Node that detects ArUco markers and publishes their pose.
    """

    def __init__(self):
        super().__init__('aruco_detect')

        self.camera_img = None

        self.declare_parameter('camera.camera_matrix')
        self.camera_matrix_vec = np.array(self.get_parameter('camera.camera_matrix').value)
        self.camera_matrix = np.reshape(self.camera_matrix_vec, (3,3))
        self.declare_parameter('camera.camera_distortion_coefficients')
        self.camera_distortion_coefficients = np.array(self.get_parameter('camera.camera_distortion_coefficients').value)

        self.bridge = CvBridge()

        self.declare_parameter('aruco.aruco_set')
        self.declare_parameter('aruco.aruco_size')
        self.declare_parameter('aruco.aruco_dictionary')
        self.declare_parameter('aruco.target_tags')
        self.declare_parameter('topic_name')

        self.which_aruco_set = self.get_parameter('aruco.aruco_set').value
        self.aruco_size = self.get_parameter('aruco.aruco_size').value
        self.which_aruco_dict = self.get_parameter('aruco.aruco_dictionary').get_parameter_value().string_value
        self.target_tag = self.get_parameter('aruco.target_tags').value
        self.which_topic = self.get_parameter('topic_name').value

        self.get_logger().info(f'where to publish: {self.which_topic}')

        dictionary_id = cv2.aruco.__getattribute__(self.which_aruco_dict)

        # Make sure the dictionary id is valid:
        try:
            if type(dictionary_id) != type(cv2.aruco.DICT_5X5_250):
                raise AttributeError
        except AttributeError:
            self.get_logger().error("bad aruco_dictionary_id: {dictionary_id}")

        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        self.get_logger().info(f'which dict: {dictionary_id}')

        #self.pub_tags = self.create_publisher(
        #        FiducialTransformArray,
        #        self.which_topic, 10)

        self.parameters = aruco.DetectorParameters_create()
        self.parameters.adaptiveThreshConstant = 10

        self.sub_test = self.create_subscription(
            Image, '/camera/image_raw',
            self.imageCallback, qos_profile_sensor_data)

        self.br = tf2_ros.TransformBroadcaster(self)

        self.win_name = f'objects detected: {self.which_aruco_set}'
        cv2.namedWindow(self.win_name)
        cv2.moveWindow(self.win_name, 40, 30)
        self.cameraAngle = 0.0

    """
    Function to generate a rotation matrix
    @param: self
    @param: axis - rotation axis, should be: 'x', 'y' or 'z'
    @param: angle [degrees] - angle of rotation
    @result: R - 4x4 homogeneous rotation matrix
    """
    def rotateHomogeneous(self, axis, angle):
        a = np.deg2rad(angle)
        R = np.eye(4)
        if axis == 'x':
            R = np.array([[1,0,0,0], \
                    [0, np.cos(a), -np.sin(a), 0],\
                    [0, np.sin(a), np.cos(a), 0], \
                    [0, 0, 0, 1]])
        elif axis == 'y':
            R = np.array([[np.cos(a),0,np.sin(a),0], \
                    [0, 1, 0, 0],\
                    [-np.sin(a), 0, np.cos(a), 0], \
                    [0, 0, 0, 1]])

        elif axis == 'z':
            R = np.array([[np.cos(a), -np.sin(a), 0,0], \
                    [np.sin(a), np.cos(a), 0, 0],\
                    [0, 0, 1, 0], \
                    [0, 0, 0, 1]])

        return R

    def imageCallback(self, image):
        """Image subscriber callback.

        Detects ArUco tags in an image. Visualizes the
        result. Publishes the tag pose. Broadcasts the tag tf.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            self.get_logger().info('CvBridgeError: ' + str(e))

        self.camera_img = cv_image

        self.camera_img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(
                self.camera_img_gray, self.aruco_dict, parameters=self.parameters)

        #self.get_logger().info(f'ids: {ids}')

        # self.get_logger().info(f'corners: {corners[0]}')
        # self.get_logger().info(f'rejected points: {rejectedImgPoints}')

        if ids is not None and len(ids) > 0:

            # ar_array = FiducialTransformArray()
            # ar_array.header.stamp = self.get_clock().now().to_msg()
            # ar_array.header.frame_id = 'camera'
            aruco.drawDetectedMarkers(self.camera_img, corners, ids)

            #--- 180 deg rotation matrix around the x axis
            R_flip  = np.zeros((3,3), dtype=np.float32)
            R_flip[0,0] = 1.0
            R_flip[1,1] =-1.0
            R_flip[2,2] =-1.0

            for i in range(len(ids)):
                ret = aruco.estimatePoseSingleMarkers(
                    corners[i],
                    self.aruco_size,
                    self.camera_matrix,
                    self.camera_distortion_coefficients)
                tagID = ids[i][0]
                if tagID == 0:
                    rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                    R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])

                    #-- Obtain the rotation matrix camera->tag
                    R_tc    = R_ct.T
                    R_tc    = R_flip*R_tc

                    #-- Get Robot's position in Tag Frame
                    pos_camera = -R_tc * np.matrix(tvec).T

                    #-- Get Robot's position in world Frame
                    #---- get defined position of the tag

                    orientation1 = self.rotateHomogeneous('y', 90)
                    orientation2 = self.rotateHomogeneous('x', -90)
                    #ic(pos_camera)
                    pos_world = orientation1[0:3,0:3] * pos_camera
                    pos_world = orientation2[0:3,0:3] * pos_world

                    aruco.drawAxis(
                        self.camera_img,
                        self.camera_matrix,
                        self.camera_distortion_coefficients,
                        rvec,
                        tvec,
                        self.aruco_size/2.0)

                    # use rodrigues() to convert rotation vector to rotation matrix
                    # tag_rot_mat = R
                    # tag_q = transforms3d.quaternions.mat2quat(tag_rot_mat)
                    #
                    # self.broadcastTagsTF(tag_q, tvec, tagID)

                    ic(pos_world)
                    #ar_tag = self.tagMsg(tag_q, tvecs[idx[0]][0], tagID)

                    #ar_array.transforms.append(ar_tag)

            #self.pub_tags.publish(ar_array)

        cv2.imshow(
            self.win_name,
            self.camera_img)
        cv2.waitKey(1)

    # def estimatePose(self):



    # def tagMsg(self, q, tvec, tagID):
    #     """ populates fiducial array message, returns msg object"""
    #     ar_tag = FiducialTransform()
    #     ar_tag.fiducial_id = np.int(tagID)
    #     ar_tag.transform.translation.x = tvec[0]
    #     ar_tag.transform.translation.y = tvec[1]
    #     ar_tag.transform.translation.z = tvec[2]
    #     ar_tag.transform.rotation.w = q[0]
    #     ar_tag.transform.rotation.x = q[1]
    #     ar_tag.transform.rotation.y = q[2]
    #     ar_tag.transform.rotation.z = q[3]
    #     return ar_tag

    def broadcastTagsTF(self, q, tvec, tagID):
        tf_tag = TransformStamped()
        tf_tag.header.stamp = self.get_clock().now().to_msg()
        tf_tag.header.frame_id = 'camera'

        tf_tag.child_frame_id = 'tag_' + str(tagID)
        tf_tag.transform.translation.x = tvec[0]
        tf_tag.transform.translation.y = tvec[1]
        tf_tag.transform.translation.z = tvec[2]
        tf_tag.transform.rotation.w = q[0]
        tf_tag.transform.rotation.x = q[1]
        tf_tag.transform.rotation.y = q[2]
        tf_tag.transform.rotation.z = q[3]
        self.br.sendTransform(tf_tag)


def main(args=None):
    rclpy.init(args=args)
    aruco_detect = ArucoDetect()
    rclpy.spin(aruco_detect)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    aruco_detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
