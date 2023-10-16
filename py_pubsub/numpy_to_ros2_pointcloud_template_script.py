'''
Alias: numpyRos2Converter
Progress: complete
'''
''''''
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class NumpyToROS2PointCloud(Node):

    def __init__(self, points, colors):
        super().__init__('numpy_to_ros2_pointcloud')
        self.publisher_ = self.create_publisher(PointCloud2, 'converted_cloud', 10)
        self.points = points
        self.colors = colors
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        header = Header()
        header.frame_id = 'map'
        header.stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

    # Pack RGB values into a single 32-bit integer
        rgb_integers = []
        for color in self.colors:
            r, g, b = color
            rgb = (int(r) << 16) | (int(g) << 8) | int(b)
            rgb_integers.append(rgb)

    # Combine this with the points to create the cloud_data array
        cloud_data = np.hstack((self.points, np.array(rgb_integers).reshape(-1, 1))).astype(np.float32).tolist()
        cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)
        self.publisher_.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)

    points = np.array([
        [0, 0, 0],
        [1, 0, 0],
        [1, 1, 0],
        [0, 1, 0],
        [0, 0, 1],
        [1, 0, 1],  
        [1, 1, 1], 
        [0, 1, 1]
    ])

    colors = np.array([
        [255, 0, 0],
        [0, 255, 0],
        [0, 0, 255],
        [0, 255, 255],  
        [255, 255, 0],  
        [255, 0, 255], 
        [0, 0, 0],  
        [0, 0, 0] 
    ])

    node = NumpyToROS2PointCloud(points, colors)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
