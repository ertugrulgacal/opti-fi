import os
import open3d as o3d
import numpy as np

main_folder_path = "/Users/egkubo/Stanford3DISD/Stanford3dDataset_v1.2_Aligned_Version"
areas_to_read = ["Area_1", "Area_2"]

def read_room(room_path):
    combined_data = np.empty((0, 6))  # (X, Y, Z, R, G, B)

    for filename in os.listdir(room_path):
        if filename.endswith(".txt"):
            txt_file_path = os.path.join(room_path, filename)
            
            data = np.loadtxt(txt_file_path)  # Read txt
            combined_data = np.concatenate((combined_data, data), axis=0)  # Combine txt files

    return combined_data

def read_areas(main_folder, area_names):
    all_data = np.empty((0, 6))  # (X, Y, Z, R, G, B)

    for area_name in area_names:
        area_path = os.path.join(main_folder, area_name)
        
        if os.path.isdir(area_path):  # Check if area folder is a directory
            for room_folder in os.listdir(area_path):
                room_path = os.path.join(area_path, room_folder)

                if os.path.isdir(room_path):  # Check if room folder is a directory
                    combined_data = read_room(room_path)
                    all_data = np.concatenate((all_data, combined_data), axis=0)  # Combine point cloud data returned from the read_room function

    return all_data

def create_point_cloud_data(main_folder, area_names):
    combined_data = read_areas(main_folder, area_names)

    # Get first 3 indices which are (X, Y, Z)
    points = combined_data[:, :3]

    # Define a point cloud object and fill it with data we have read
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

pcd = create_point_cloud_data(main_folder_path, areas_to_read)

# Visualization
o3d.visualization.draw_geometries([pcd])