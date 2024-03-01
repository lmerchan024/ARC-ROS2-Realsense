'''
alias:numpyO3dConverter
progress:complete
'''
import numpy as np
import open3d as o3d

# points of a cube of width 1
points = np.array([
    [0, 0, 0],  # Vertex 1
    [1, 0, 0],  # Vertex 2
    [1, 1, 0],  # Vertex 3
    [0, 1, 0],  # Vertex 4
    [0, 0, 1],  # Vertex 5
    [1, 0, 1],  # Vertex 6
    [1, 1, 1],  # Vertex 7
    [0, 1, 1]   # Vertex 8
])

# Corresponding colors
colors = np.array([
    [1, 0, 0],  # Red
    [0, 1, 0],  # Green
    [0, 0, 1],  # Blue
    [0, 1, 1],  # Cyan
    [1, 1, 0],  # Yellow
    [1, 0, 1],  # Magenta
    [0, 0, 0],  # Black
    [0, 0, 0]   # Black
])

# Convert numpy array to Open3D point cloud
point_cloud = o3d.geometry.PointCloud()#created empty o3d pointcloud

point_cloud.points = o3d.utility.Vector3dVector(points)#input:(n,3)numpy array, store it as a o3d array of shape (n,3) into the '.points' property of pcd
print(np.asarray(point_cloud.points))
point_cloud.colors = o3d.utility.Vector3dVector(colors)
print(np.asarray(point_cloud.colors))

#visualize pointcloud
o3d.visualization.draw_geometries([point_cloud])