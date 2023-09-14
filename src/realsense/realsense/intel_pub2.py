import rclpy #import ROS Client Library for Python
from rclpy.node import Node #needed for node creation
from sensor_msgs.msg import Image #import built-in Image msg type that the node uses to structure the data passing the topic
from cv_bridge import CvBridge #import the CvBridge class from the cv_bridge pkg. which opffers methods to facilitate conversion, read below
#primary role of cv_bridge is to enable the conversion of ROS image msg, from sensor_imgs.msg, to OpenCV image formats and vice versa

from sensor_msgs.msg import PointCloud2 #Import the 'PointCloud2' message type
import sensor_msgs.point_cloud2 as pc2 #this module provides utility functions to read and write 'PointCloud2' messages

import cv2
import pyrealsense2 as rs 
import numpy as np

class IntelPublisher(Node):
    def __init__(self):
        super().__init__("intel_publisher")
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frames", 10) #data type is 'Image', topic name='rgb frames', qeue list of 10 send frames?

        self.intel_publisher_pc = self.create_publisher(PointCloud2, "pointcloud_data", 10)

        timer_period = 0.05 #every 0.05sec, run the code back that will send a frame
        self.br_rgb = CvBridge() #convert from CvBridge image format to sendor_msgs.msg Image format

        try:
            
            self.pipe = rs.pipeline() #connect to camera
            self.cfg  = rs.config()
            self.cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)#stream color data, 640x480, format=bgr8,30fps
            self.pipe.start(self.cfg)#start camera
            self.timer = self.create_timer(timer_period, self.timer_callback) #timer_callback fun push frame to topic
        except Exception as e:
            print(e) #print error
            self.get_logger().error("INTEL REALSENSE IS NOT CONNECTED") #display error

    def timer_callback(self): #create timer
        frames = self.pipe.wait_for_frames() #grab frames
        color_frame = frames.get_color_frame() #color frames
        color_image = np.asanyarray(color_frame.get_data()) #change color frame into color image

        self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image)) #convert cv2 image dara to Image data
        self.get_logger().info("Publishing rgb frame")


def main(args = None): #main fucntion
    rclpy.init(args = None)
    intel_publisher = IntelPublisher()
    rclpy.spin(intel_publisher)#what does this do? what is spin?
    intel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()