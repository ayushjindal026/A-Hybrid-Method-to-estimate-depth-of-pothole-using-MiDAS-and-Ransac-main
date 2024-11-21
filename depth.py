import open3d as o3d
import numpy as np

# Load point cloud
pcd = o3d.io.read_point_cloud(r"C:\Users\Shashank\OneDrive\Desktop\Point Cloud\DATA\output\potholes59.ply")

# Preprocess the point cloud (e.g., voxel downsampling)
pcd = pcd.voxel_down_sample(voxel_size=0.05)

# Segment the road surface using RANSAC plane fitting
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
road_surface = pcd.select_by_index(inliers)
pothole_points = pcd.select_by_index(inliers, invert=True)

# Get the z-values of the remaining points (likely potholes)
pothole_z = np.asarray(pothole_points.points)[:, 2]  # Extract z-coordinates

# Estimate pothole depth
pothole_depth = np.min(pothole_z) - np.median(np.asarray(road_surface.points)[:, 2])

print(f"Estimated Pothole Depth: {abs(pothole_depth)} meters")

# Visualize
o3d.visualization.draw_geometries([road_surface, pothole_points])
