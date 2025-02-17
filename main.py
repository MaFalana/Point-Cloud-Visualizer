import numpy as np
import laspy as lp
import open3d as o3d
import os



# load the COLORIZED point cloud

point_cloud = lp.read('./CraneBuildingForVeiwing.laz')

print([dimension.name for dimension in point_cloud.point_format.dimensions])
print(np.max(point_cloud.red))

# Convert Point cloud (laspy object) to numpy object
points = np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).transpose()
colors = np.vstack((point_cloud.red, point_cloud.green, point_cloud.blue)).transpose()

factor = 10  # downsample factor
decimated_points_random = points[::factor] # takes point every factor
decimated_colors_random = colors[::factor]

print("Points Number Before: ", len(points))
print("Points Number After: ", len(decimated_points_random))

# 6 - Pre processing 3D



# 7 - Visualization strategy
pcd = o3d.geometry.PointCloud() #generate geometry
pcd.points = o3d.utility.Vector3dVector(decimated_points_random) # pass the points attributes
pcd.colors = o3d.utility.Vector3dVector(decimated_colors_random/65535) # pass the colors attributes make b/t 0 and 1

#plot point cloud
o3d.visualization.draw_geometries([pcd]) 




if __name__ == '__main__':
    main()