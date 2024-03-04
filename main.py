import os
import open3d as o3d
import numpy as np

# Set the path to the main folder
main_folder = "/Users/egkubo/Stanford3DISD/Stanford3dDataset_v1.2_Aligned_Version"

# Function to combine point clouds from multiple text files in a folder
def combine_point_clouds(folder_path):
    combined_data = np.empty((0, 6))  # Assuming 6 columns (X, Y, Z, R, G, B)

    # Loop through files in the folder
    for filename in os.listdir(folder_path):
        if filename.endswith(".txt"):
            file_path = os.path.join(folder_path, filename)
            data = np.loadtxt(file_path)
            combined_data = np.concatenate((combined_data, data), axis=0)

    return combined_data

# Initialize an empty array to store the combined point cloud data
all_data = np.empty((0, 6))

# Combine point clouds for all rooms in "Area_1"
area_path = os.path.join(main_folder, "Area_1")

# Check if "Area_1" is a directory
if os.path.isdir(area_path):
    for room_folder in os.listdir(area_path):
        room_path = os.path.join(area_path, room_folder)

        # Check if the room folder is a directory
        if os.path.isdir(room_path):
            # Combine point clouds
            combined_data = combine_point_clouds(room_path)
            all_data = np.concatenate((all_data, combined_data), axis=0)

# Extract point cloud coordinates from the combined data
points = all_data[:, :3]

# Create Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Visualize the combined point cloud
o3d.visualization.draw_geometries([pcd])