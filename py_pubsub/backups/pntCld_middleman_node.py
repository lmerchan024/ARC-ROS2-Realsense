'''
This node subscribes to the realsense camera node pc topic
and sends it back to rviz2 with no manipulation
alias is pntCldMidMan
'''
#%%
import rclpy
from rclpy.node import Node
import open3d as o3d
import numpy as np

#from std_msgs.msg import String
import sensor_msgs.msg as sensor_msgs
import std_msgs
#%% Node Creation
class PntCld2SubPub(Node):

    def __init__(self):
        super().__init__('pntCldMidMan') #node name
        
        #listen to the topic '/camera/depth/color/points' and send it to the 'listener_callback' function
        self.subscription = self.create_subscription(sensor_msgs.PointCloud2,'/camera/depth/color/points',self.listener_callback,10)
        self.subscription  # prevent unused variable warning?

        #create publisher
        self.publisher_=self.create_publisher(sensor_msgs.PointCloud2, 'pntCld2',10) #create new topic called 'pntCld2'


    #Functions that will be called upon
    def listener_callback(self, pntCld2):
        print('recieiving PointCloud2 data')
        self.publisher_.publish(pntCld2)
        print('Publishing same data')
        #pntCld2.<specific_data>
        #https://docs.ros2.org/latest/api/sensor_msgs/msg/PointCloud2.html
        #uint8[] data is an array

#%% Running the node
def main(args=None):
    rclpy.init(args=args)

    pntCld2_intermediary=PntCld2SubPub() #call upon the node
    rclpy.spin(pntCld2_intermediary)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pntCld2_intermediary.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
#%%