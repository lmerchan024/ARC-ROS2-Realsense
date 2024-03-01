'''
This node creates and publishes a pointcloud of a wave on a plan which can be visualized in rviz2
This node will serve as reference for creating the converter midman node
alias is pntCldCreator

No more edits will be done to this script. it is for template use only

COMPLETED oct10
'''
#%%
'''
This node is an upgrade of the pntCld_middleman_node
It will listen to pc data, convert it to o3d pc, revert back to ros2 pc data, and publish this to rviz2
WIP
alias is pntCldConverter

import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
import sensor_msgs.msg as sensor_msgs
import std_msgs
#%% Node Creation
class PntCld2SubPub(Node):

    def __init__(self):
        super().__init__('pntCldHandler') #node name
        
        #listen to the topic '/camera/depth/color/points' and send it to the 'listener_callback' function
        self.subscription = self.create_subscription(sensor_msgs.PointCloud2,'/camera/depth/color/points',self.listener_callback,10)
        self.subscription  # prevent unused variable warning?

        #create publisher
        self.publisher_=self.create_publisher(sensor_msgs.PointCloud2, 'new_pntCld2',10) #create new topic called 'new_pntCld2'


    #Functions that will be called upon
    def listener_callback(self, pntCld2):
        self.publisher_.publish(pntCld2)

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
'''

import numpy as np

'''
Creating pointcloud from scracth

'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class PointCloudPublisher(Node):

    rate = 30
    moving = True
    width = 100
    height = 100

    header = Header()
    header.frame_id = 'map'

    dtype = PointField.FLOAT32
    point_step = 16
    fields = [PointField(name='x', offset=0, datatype=dtype, count=1),
              PointField(name='y', offset=4, datatype=dtype, count=1),
              PointField(name='z', offset=8, datatype=dtype, count=1),
              PointField(name='intensity', offset=12, datatype=dtype, count=1)]

    def __init__(self):
        super().__init__('pc_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'test_cloud', 10)
        timer_period = 1 / self.rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.header.stamp = self.get_clock().now().to_msg()
        x, y = np.meshgrid(np.linspace(-2, 2, self.width), np.linspace(-2, 2, self.height))
        z = 0.5 * np.sin(2*x-self.counter/10.0) * np.sin(2*y)
        points = np.array([x, y, z, z]).reshape(4, -1).T
        pc2_msg = point_cloud2.create_cloud(self.header, self.fields, points)
        self.publisher_.publish(pc2_msg)

        if self.moving:
            self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    pc_publisher = PointCloudPublisher()
    rclpy.spin(pc_publisher)
    pc_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()