'''
alias: ros2NumpyConverter
progress: COMPLETE? i think, can extract rgb and xyz, and i think it gets all other PointCloud2 Properties
'''

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

def convertPointCloud2MsgToNumpy(msg):
    # Extract the xyz and rgb fields from the PointCloud2 message
    # xyz_data = point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
    #print('namedtuples contaniing values for each point:',sensor_msgs_py.point_cloud2.read_points_list())
    #print('msg.fields=',msg.fields)
    points_list = point_cloud2.read_points_list(msg, field_names=("rgb",), skip_nans=True)

    rgb_packed = np.array([point.rgb for point in points_list])

    rgb_data = np.c_[((rgb_packed >> 16) & 0xFF).astype(np.uint8),((rgb_packed >> 8) & 0xFF).astype(np.uint8),(rgb_packed & 0xFF).astype(np.uint8)]
    #convert rgb_data into the same (8,3) array form of the xyz array. the current form is a 2 d array
    converted_rgb_data = np.array([(r, g, b) for r, g, b in rgb_data], 
                            dtype=[('r', 'u1'), ('g', 'u1'), ('b', 'u1')])

    # Store the other properties of the PointCloud2 message
    stored_header = msg.header
    stored_height = msg.height
    stored_width = msg.width
    stored_fields = msg.fields
    stored_is_bigendian = msg.is_bigendian
    stored_point_step = msg.point_step
    stored_row_step = msg.row_step
    stored_is_dense = msg.is_dense

    print("XYZ Data:", xyz_data)
    print("RGB Data:", converted_rgb_data)
    return xyz_data, converted_rgb_data

class ROS2PointCloudToNumpy(Node):
    def __init__(self):
        super().__init__('ros2_pointcloud_to_numpy')
        # Listen to the topic 'converted_cloud' and send it to the 'listener_callback' function
        self.subscription = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.listener_callback, 10)
    def listener_callback(self, msg):
        (xyz, rgb) = convertPointCloud2MsgToNumpy(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ROS2PointCloudToNumpy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
