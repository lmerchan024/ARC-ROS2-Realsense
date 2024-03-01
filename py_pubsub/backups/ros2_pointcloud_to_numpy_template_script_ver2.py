'''
alias: ros2_pointcloud_to_numpy_template_script_ver2
progress: COMPLETE? i think, can extract rgb and xyz, and i think it gets all other PointCloud2 Properties
'''

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class ROS2PointCloudToNumpy(Node):
    
    def __init__(self):
        super().__init__('ros2_pointcloud_to_numpy')
        
        # Listen to the topic 'converted_cloud' and send it to the 'listener_callback' function
        #self.subscription = self.create_subscription(PointCloud2, 'converted_cloud', self.listener_callback, 10)
        self.subscription = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.listener_callback, 10)

    def listener_callback(self, msg):
        # Extract the xyz and rgb fields from the PointCloud2 message
        
        xyz_data = point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        #print('namedtuples contaniing values for each point:',sensor_msgs_py.point_cloud2.read_points_list())
        #print('msg.fields=',msg.fields)
        points_list = point_cloud2.read_points_list(msg, field_names=("rgb",), skip_nans=True)

        # Unpack RGB data using the custom function
        rgb_colors = np.array([self.unpack_rgb_from_float(point.rgb) for point in points_list]) / 255.0
        
        print("XYZ Data:", xyz_data)
        print("RGB Data:", rgb_colors)

    @staticmethod
    def unpack_rgb_from_float(packed_rgb_float):
        # Unpack a float32 into an array of R, G, B values.
        rgb_uint32 = np.frombuffer(np.array([packed_rgb_float], dtype=np.float32).tobytes(), dtype=np.uint32)[0]
        r = (rgb_uint32 >> 16) & 0x0000ff
        g = (rgb_uint32 >> 8) & 0x0000ff
        b = (rgb_uint32) & 0x0000ff
        return np.array([r, g, b], dtype=np.uint8)


def main(args=None):
    rclpy.init(args=args)
    node = ROS2PointCloudToNumpy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
