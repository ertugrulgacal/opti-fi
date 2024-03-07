import open3d as o3d
import numpy as np

def read_single_file(file_path):
    try:
        data = np.loadtxt(file_path)  # Read txt
        return data
    except ValueError as e:
        print(f"Error reading {file_path}: {e}. Returning empty array.")
        return np.empty((0, 6))  # Return empty array if there is an error

def create_point_cloud_from_file(file_path):
    data = read_single_file(file_path)

    # Get first 3 indices which are (X, Y, Z)
    points = data[:, :3]

    # Get last 3 indices which are (R, G, B)
    colors = data[:, 3:]

    # Define a point cloud object and fill it with data we have read
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)  # Normalize color values to [0, 1]

    return pcd

# Example usage
single_file_path = "teknofest_ornek_data_v1/ornek_ev_1_lidar_export.xyz"
pcd_single = create_point_cloud_from_file(single_file_path)

# Visualization
o3d.visualization.draw_geometries([pcd_single])
