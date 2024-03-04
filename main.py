import os
import open3d as o3d
import numpy as np

main_folder_path = "/Users/egkubo/Stanford3DISD/Stanford3dDataset_v1.2_Aligned_Version"
areas_to_read = ["Area_1"]
rooms_to_read = ["office_1", "office_2", "office_3", "office_4", "office_5", "office_6"]

def read_room(room_path):
    combined_data = np.empty((0, 6))  # (X, Y, Z, R, G, B)

    for filename in os.listdir(room_path):
        if filename.endswith(".txt") and filename[:-4] in rooms_to_read:
            txt_file_path = os.path.join(room_path, filename)

            try:
                data = np.loadtxt(txt_file_path)  # Read txt
                combined_data = np.concatenate((combined_data, data), axis=0)  # Combine txt files
            except ValueError as e:
                print(f"Error reading {txt_file_path}: {e}. Skipping.")

    return combined_data

def read_areas(main_folder, area_names):
    all_data = np.empty((0, 6))  # (X, Y, Z, R, G, B)

    for area_name in area_names:
        area_path = os.path.join(main_folder, area_name)

        if os.path.isdir(area_path):  # Check if area folder is a directory
            for room_folder in os.listdir(area_path):
                room_path = os.path.join(area_path, room_folder)

                if os.path.isdir(room_path) and room_folder in rooms_to_read:  # Check if room folder is a directory and in the specified list
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