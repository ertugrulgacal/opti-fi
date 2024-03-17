import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

FILE_PATH = "teknofest_ornek_data_v1/ornek_ev_1_lidar_export.xyz"

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

def replace_values(points, threshold=0.5):
    # Create a new list to store filtered points
    filtered_points = []

    # Iterate through each point
    for point in points:
        # Check if the Z value is greater than or equal to the threshold
        if point[2] >= threshold:
            # Add the point to the filtered list
            filtered_points.append(point)

    return filtered_points

def create_point_cloud_from_list(point_cloud):
    points = [[point.x, point.y, point.z] for point in point_cloud]

    points = replace_values(points)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

def read_point_cloud_data(file_path):
    point_cloud = []

    with open(file_path, 'r') as file:
        for line in file:
            # Split the line into components and convert to float
            components = line.split()
            x = float(components[0])
            y = float(components[1])
            z = float(components[2])

            # Create a Point object and add it to the point cloud list
            point_cloud.append(Point(x, y, z))

    return point_cloud

def group_points(point_cloud, big_threshold=0.1, small_threshold=0.0001):  # Slow and doesnt work
    grouped_points = []

    # Initialize a set to keep track of assigned points
    assigned_points = set()

    # Group points based on x coordinates
    for i, point1 in enumerate(point_cloud):
        # Skip points that are already assigned to a group
        if i in assigned_points:
            continue
        
        # Create a new group for the current point
        group = [point1]

        for j, point2 in enumerate(point_cloud[i+1:], start=i+1):
            # Check if the points have the same x value and y value within the thresholds
            if (abs(point1.x - point2.x) < small_threshold and abs(point1.y - point2.y) < big_threshold) or \
               (abs(point1.y - point2.y) < small_threshold and abs(point1.x - point2.x) < big_threshold):
                group.append(point2)
                # Mark the point as assigned
                assigned_points.add(j)

        # Add the group to the list of grouped points
        grouped_points.append(group)

    return grouped_points

def main():
    point_cloud_data = read_point_cloud_data(FILE_PATH)

    #grouped_points = group_points(point_cloud_data)
    
    pcd = create_point_cloud_from_list(point_cloud_data)
    o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    main()
