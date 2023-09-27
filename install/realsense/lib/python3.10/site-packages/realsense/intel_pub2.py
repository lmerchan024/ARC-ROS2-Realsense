import rclpy #import ROS Client Library for Python
from rclpy.node import Node #needed for node creation
from sensor_msgs.msg import Image #import built-in Image msg type that the node uses to structure the data passing the topic
from cv_bridge import CvBridge #import the CvBridge class from the cv_bridge pkg. which opffers methods to facilitate conversion, read below
#primary role of cv_bridge is to enable the conversion of ROS image msg, from sensor_imgs.msg, to OpenCV image formats and vice versa

from sensor_msgs.msg import PointCloud2 as pc2 #Import the 'PointCloud2' message type
#%%THIS MODULE IS FOR ROS1 AND ISN"T COMPATABLE WITH THIS CODE
import sensor_msgs.point_cloud2 as pc2 #this module provides utility functions to read and write 'PointCloud2' messages
#%%
import cv2s
import pyrealsense2 as rs 
import numpy as np

class IntelPublisher(Node):
    def __init__(self):
        super().__init__("intel_publisher")
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frames", 10) #data type is 'Image', topic name='rgb frames', qeue list of 10 send frames?

        self.intel_publisher_pc = self.create_publisher(PointCloud2, "pointcloud_data", 10) #data type is 'PointCloud2', topic name=pointcloud_data', quality of service =10, what is qos?

        timer_period = 0.05 #every 0.05sec, run the code back that will send a frame
        self.br_rgb = CvBridge() #convert from CvBridge image format to sendor_msgs.msg Image format

        try:
            
            self.pipe = rs.pipeline() #connect to camera
            self.cfg  = rs.config()
            self.cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)#stream color data, 640x480, format=bgr8,30fps
            self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30) #stream depth data, 640x480, format=z16, 30fps
            self.pipe.start(self.cfg)#start camera
            self.timer = self.create_timer(timer_period, self.timer_callback) #timer_callback fun push frame to topic
        except Exception as e:
            print(e) #print error
            self.get_logger().error("INTEL REALSENSE IS NOT CONNECTED") #display error

    def timer_callback(self): #create timer
        frames = self.pipe.wait_for_frames() #grab frames

        depth_frames=frames.get_depth_frames() #grab depth frames
        color_frames = frames.get_color_frame() #color frames

        pc=rs.pointcloud() #create pointcloud object, this is empty at first
        pointcloud_data=pc.calculate(depth_frames)#from the depth frame data, calculate the pointcloud data
        vtx = np.asanyarray(pointcloud_data.get_vertices()) #vertex=numpy array of all data points in the pointcloud.
        points = np.array(vtx, dtype=np.float32).reshape((-1, 3))  #convert array data into np.float32 data type. If vtx is already this type, this line is redundant. idk if this is the case.
        #reshape array such that each vertex/point in the pointcloud is represetned as a vecotr of x,y,z coords
        #idk why the -1 is there, but the reshape function is saying 'reshape this array such that it has 3 clmns and however many rows are needed (-1) to maintain the same number of elmnt'
        pc2_msg = pc2.create_cloud_xyz32(depth_frames.get_profile().as_video_stream_profile().intrinsics, points) #convert the 'points' array into the 'PointCloud2' data type from the sensor_msgs.msg module,no idea what the nuances of this line does
        self.intel_publisher_pc.publish(pc2_msg)# publish pc2_msg
        
        color_image = np.asanyarray(color_frames.get_data()) #change color frame into color image
        self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image)) #convert cv2 image dara to Image data and publish

        self.get_logger().info("Publishing rgb frame and pointcloud")


def main(args = None): #main fucntion
    rclpy.init(args = None)
    intel_publisher = IntelPublisher()
    rclpy.spin(intel_publisher)#what does this do? what is spin?
    intel_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()