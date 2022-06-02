#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
# to load the video
import os
video_path = os.path.expanduser('~/dev_ws/src/robominer/robominer_locomotion_control/config/') # <-- use package path
video_file = 'output3.mp4'
video = os.path.join(video_path, video_file)
if os.path.exists(video):
    print(f"video ok!")
else:
    print(f"no video in: {video}")

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    super().__init__('image_publisher')

    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)

    timer_period = 1.0  # seconds

    self.timer = self.create_timer(timer_period, self.timer_callback)

    self.cap = cv2.VideoCapture(video)
    if (self.cap.isOpened()== False):
        self.get_logger().info("Error opening video stream or file")

    self.br = CvBridge()

  def timer_callback(self):

    ret, frame = self.cap.read()
    # self.get_logger().info(f'ret: {ret}')

    if ret == False:
        self.get_logger().info(f'no more frames... killin node')
        self.destroy_node()

    if ret == True:
    #   self.get_logger().info(f'got frame')

      self.get_logger().info('Publishing video frame')
      # Publish a lower res image.
      #cv_image = self.br.cv2_to_imgmsg(cv2.resize(frame, (0,0), fx=1.0, fy=1.0), "bgr8")
      cv_image = self.br.cv2_to_imgmsg(frame, "bgr8")
      self.publisher_.publish(cv_image)



def main(args=None):
  rclpy.init(args=args)

  image_publisher = ImagePublisher()

  rclpy.spin(image_publisher)

  image_publisher.destroy_node()

  rclpy.shutdown()

if __name__ == '__main__':
  main()
