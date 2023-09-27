import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import cv2
import open3d as o3d
import numpy as np


class IntelSubscriber(Node): #create intel subscriber class
    def __init__(self):
        super().__init__("intel_subscriber")
        #sub to image topic and pointcloud topic
        self.subscription_rgb = self.create_subscription(Image, "/camera/color/image_raw", self.rgb_frame_callback, 10)
        ##self.subscription_pointcloud2 = self.create_subscription(PointCloud2, "/camera/depth/color/points", self.pointcloud2_callback, 10) #this line doesnt work, idk why
        #bridge between cv and ros for rgb images (needed to handle image data using opencv)
        self.br_rgb = CvBridge() #bridge between cv and ros?


    def rgb_frame_callback(self, data): #define rgb frame callback, takes in data
        self.get_logger().info("Receiving RGB frame")
        current_frame = self.br_rgb.imgmsg_to_cv2(data) #convert msg to cv2 data
        cv2.imshow("RGB", current_frame) #then display that data
        cv2.waitKey(1)

    def pointcloud2_callback(self, data): #define rgb frame callback, takes in data
        self.get_logger().info("Receiving pointcloud2 frame") #
        current_frame = self.br_rgb.imgmsg_to_cv2(data) #convert msg to cv2 data
        cv2.imshow("RGB", current_frame) #then display that data
        cv2.waitKey(1)




def main(args = None): #main function
    rclpy.init(args = args)
    intel_subscriber = IntelSubscriber()
    rclpy.spin(intel_subscriber) #what does this do? what is spin?
    intel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()