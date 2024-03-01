'''
alias: pntCldConverter

This is an upgrade of the midman node that will convert between pntcld data types
It will use parts of the old midman node and the pntCld_creator_example_node.py


WIP Oct10,13

Need to work on these elements
pc>numpy
numpy>o3d completed by 
o3d>numpy
numpy>pc2
'''
'''
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

'''

'''
class PointCloudConverter(Node):
    
    def __init__(self):
        super().__init__('pntCldConverter') #node name
        
        #listen to the topic '/camera/depth/color/points' and send it to the 'listener_callback' function
        self.subscription = self.create_subscription(PointCloud2,'/camera/depth/color/points',self.listener_callback,10)
        self.subscription  # prevent unused variable warning?

        #create publisher
        self.publisher_=self.create_publisher(PointCloud2, 'pntCld2',10) #create new topic called 'pntCld2'


    #Functions that will be called upon
    def listener_callback(self, pntCld2):
        #ROS2 PointCloud2 > numpy
        print("Receiving a PointCloud2 message!")
        np_points = point_cloud2.read_points(pntCld2, skip_nans=True)
        print(np_points.dtype)
        print(np_points)



        
        self.publisher_.publish(pntCld2)
        

#%% Running the node
def main(args=None):
    rclpy.init(args=args)

    pntCld2_intermediary=PointCloudConverter() #call upon the node
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
'''
import open3d as o3d
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class PointCloudConverter(Node):
    
    def __init__(self):
        super().__init__('pntCldConverter') #node name
        self.subscription = self.create_subscription(PointCloud2,'/camera/depth/color/points',self.listener_callback,10)
        self.subscription
        self.publisher_ = self.create_publisher(PointCloud2, 'pntCld2', 10)



    #this is what i need to work on!!!
    def listener_callback(self, pntCld2):
        # ROS PointCloud2 -> numpy
        print('---\nReceiving PointCloud2 data...')

        #converting PointCloud2 > numpy
        print('Converting PointCloud2 to numpy array...')
        np_points = point_cloud2.read_points(pntCld2, skip_nans=True) #skip over points that contain Not a Number data (ie missing data in cloud)
        print('Done!')
        #print(np_points.dtype)
        #print(len(np_points)) #number of points

        #getting lost data from conversion

        #converting numpy to o3d
        #get coords of all points
        print('Extracting position numpy array...')
        xyz = np_points[['x', 'y', 'z']]
        print(xyz.shape)
        print('Done!')
        #print(len(xyz)) #number of points

        print('Creating empty open3d pointcloud...')
        o3d_pc = o3d.geometry.PointCloud()
        o3d_pc.points = o3d.utility.DoubleVector(xyz)
        print('Done!')

        print(o3d_pc)


'''
'''
        #get colors of each point
        rgb_floats = np_points['rgb']

        #converting between color formats
        rgb_uint32 = np.array([rgb_floats[0]]).view(np.uint32)
        rgb_uint8 = ((rgb_uint32 >> np.array([0, 8, 16])) & 255).view(np.uint8).reshape(-1, 3)
        print("RGB of the first point:", rgb_floats[0],'=',rgb_uint8[0])
        
        #pc2 color value uses a single calue
        #o3d pc color value range 0-1

###this sec doesnt work, needs working on
'''

'''

'''
'''
'''





'''
#this is fine
def main(args=None):
    rclpy.init(args=args)

    point_cloud_converter_node = PointCloudConverter()
    rclpy.spin(point_cloud_converter_node)# Spin the node so it can process callbacks

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    point_cloud_converter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
'''
import numpy as np
import rclpy
import open3d as o3d
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class PointCloudConverter(Node):

    def __init__(self):
        super().__init__('point_cloud_converter')

        # Subscription to PointCloud2
        self.subscription = self.create_subscription(PointCloud2, '/camera/depth/color/points', self.listener_callback, 10)
        # Publisher to PointCloud2
        self.publisher_ = self.create_publisher(PointCloud2, 'reconverted_cloud', 10)

    def listener_callback(self, msg):
        # 1. Convert ROS2 PointCloud2 data to numpy
        xyz_data = np.array(list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))))
        points_list = point_cloud2.read_points_list(msg, field_names=("rgb",), skip_nans=True)
        rgb_packed = np.array([point.rgb for point in points_list])
        rgb_data = np.c_[((rgb_packed >> 16) & 0xFF).astype(np.uint8),
                         ((rgb_packed >> 8) & 0xFF).astype(np.uint8),
                         (rgb_packed & 0xFF).astype(np.uint8)]

        # 2. Convert numpy to o3d point cloud
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(xyz_data)
        point_cloud.colors = o3d.utility.Vector3dVector(rgb_data / 255)

        # 3. Visualize the o3d point cloud
        #o3d.visualization.draw_geometries([point_cloud])

        # 4. Convert o3d point cloud back to numpy
        xyz_converted = np.asarray(point_cloud.points)
        rgb_converted = (np.asarray(point_cloud.colors) * 255).astype(np.uint8)

        # 5. Convert numpy back to ROS2 PointCloud2
        header = Header()
        header.frame_id = msg.header.frame_id
        header.stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
        ]

        rgb_integers = []
        for r, g, b in rgb_converted:
            rgb = (int(r) << 16) | (int(g) << 8) | int(b)
            rgb_integers.append(rgb)

        cloud_data = np.hstack((xyz_converted, np.array(rgb_integers).reshape(-1, 1))).astype(np.float32).tolist()
        cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)
        self.publisher_.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
