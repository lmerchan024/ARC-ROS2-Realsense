'''
Alias: o3dNumpyConverter
Progress: wip
'''

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d

class O3DToNumpyNode(Node):

    def __init__(self):
        super().__init__('o3d_to_numpy_node')
        
        # Sample data: For demonstration purposes, creating a point cloud using Open3D
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector([
            [0, 0, 0],
            [1, 0, 0],
            [1, 1, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 1],  
            [1, 1, 1], 
            [0, 1, 1]
        ])
        point_cloud.colors = o3d.utility.Vector3dVector([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [0, 1, 1],  
            [1, 1, 0],  
            [1, 0, 1], 
            [0, 0, 0],  
            [0, 0, 0]
        ])

        o3d.visualization.draw_geometries([point_cloud])
        # Convert Open3D point cloud to numpy arrays
        self.xyz_array = np.asarray(point_cloud.points)
        self.rgb_array = np.asarray(point_cloud.colors)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        print("\nXYZ Array:\n", self.xyz_array)
        print("\nRGB Array:\n", self.rgb_array)


def main(args=None):
    rclpy.init(args=args)
    node = O3DToNumpyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
