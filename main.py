import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

file_path = "teknofest_ornek_data_v1/ornek_ev_1_lidar_export.xyz"

def replace_values(data):
    new_data = []

    for point in data:
        if point[2] > 0.5:
            new_data.append(point)

    return new_data

def read_single_file(file_path):
    try:
        data = np.loadtxt(file_path)  # Read txt
        return data
    except ValueError as e:
        print(f"Error reading {file_path}: {e}. Returning empty array.")
        return np.empty((0, 6))  # Return empty array if there is an error

def create_point_cloud_from_file(file_path):
    data = read_single_file(file_path)

    points = data[:, :3]  # X, Y, Z

    #colors = data[:, 3:]  # R, G, B

    wall_points = replace_values(points)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(wall_points)
    #pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)  # Normalize color values to [0, 1]

    return pcd


pcd = create_point_cloud_from_file(file_path)

# Visualization
o3d.visualization.draw_geometries([pcd])