import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
import open3d as o3d



class plane_getter(Node):
    
    def __init__(self):
        super().__init__('plane_getter')
        self.subscription = self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.listener_callback, 10)
        self.publisher_=self.create_publisher(PointStamped, 'detected_corners',10) #create new topic called 'pntCld2'

    def numpy_from_pc2(self, points_pc2):
        # Convert PointCloud2 to numpy array
        gen = point_cloud2.read_points(points_pc2, skip_nans=True, field_names=("x", "y", "z")) # using a Python gen, get the xyz coords
        points_np = np.array(list(gen)) #convert gen values to list to numpy array
        return points_np
    
    def tuples_to_matrix(self, old):
        N = len(old)  # Determine the number of elements in old
        new = np.zeros((N, 3))  # Initialize an empty 2D array of shape (N, 3)
        
        # Iterate over the structured array and explicitly extract x, y, z values
        for i in range(N):
            new[i, 0] = old[i]['x']
            new[i, 1] = old[i]['y']
            new[i, 2] = old[i]['z']
            
        return new
    
    def o3d_from_numpy(self, points_np):
        points_o3d = o3d.geometry.PointCloud() #create empty o3d object
        points_o3d.points = o3d.utility.Vector3dVector(points_np) #populate o3d object with points
        return points_o3d

    def detect_planes(self, points_o3d):

        points_o3d = points_o3d.voxel_down_sample(voxel_size=0.01) # downsample pointcloud
        points_o3d.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)) # calcluate normals for each point
        assert (points_o3d.has_normals()) #check if normals exist, else rise error

        #detect planar patches
        planes = points_o3d.detect_planar_patches(
            normal_variance_threshold_deg=30,
            coplanarity_deg=85,
            outlier_ratio=1,
            min_plane_edge_length=0,
            min_num_points=0,
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
        
        return planes

    def get_center_of_plane_of_interest(self, plane): #get center of a single plane
        center = plane.get_center()
        return center

    def get_corners_of_plane_of_interest(self, plane_of_interest): #get the corners of a single plane
        center_of_plane_of_interest = self.get_center_of_plane_of_interest(plane_of_interest)
        R = plane_of_interest.R
        extents = plane_of_interest.extent
        
        # The plane's normal is assumed to be along the axis with the smallest extent
        min_extent_idx = np.argmin(extents)
        indices = [0, 1, 2]
        indices.remove(min_extent_idx)
        
        # Compute the four corners of the plane
        corners_of_plane_of_interest = []
        for i in [-1, 1]:
            for j in [-1, 1]:
                corner_vect = np.zeros(3)
                corner_vect[indices[0]] = extents[indices[0]] / 2 * i
                corner_vect[indices[1]] = extents[indices[1]] / 2 * j
                # Ensure the corners are on the plane by making the component along the normal very small or zero
                corner_vect[min_extent_idx] = 0  # Place the corner on the plane, assuming the plane is at the center
                corner = center_of_plane_of_interest + R.dot(corner_vect)
                corners_of_plane_of_interest.append(corner)
        
        return corners_of_plane_of_interest

    def publish_corner(self, corner, frame_id): #publishes a single corner, given corner 
        # Publish a detected corner as PointStamped message
        corner_msg = PointStamped()
        corner_msg.header.stamp = self.get_clock().now().to_msg()
        corner_msg.header.frame_id = frame_id
        corner_msg.point.x = corner[0]
        corner_msg.point.y = corner[1]
        corner_msg.point.z = corner[2]
        self.publisher_.publish(corner_msg)
        
    def listener_callback(self, msg):
        PoI = np.array([0.5,0.5,0.5]) #Point Of Interest as selected by santi

        print('\nConverting PointCloud2 to Numpy (N,) array')
        points_np = self.numpy_from_pc2(msg)
        print('Done!')

        print('Converting Numpy (N,) array to Numpy (N,3) array')
        points_np = self.tuples_to_matrix(points_np)
        print('Done!')

        print('Converted Numpy (N,3) array to Open3d PointCloud')
        points_o3d = self.o3d_from_numpy(points_np)
        print('Done')

        print('Detecting Planes')
        all_planes = self.detect_planes(points_o3d)
        print('Done!')

        print('Detecting Plane of Interest')
        plane_of_interest = self.get_plane_of_interest(all_planes)
        print('Done!')

        print(corners)
        # for corner in corners:
        #     self.publish_corner(corner, msg.header.frame_id)



def main(args=None):
    rclpy.init(args=args)
    node = plane_getter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# %%