import open3d as o3d
import numpy as np


class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def create_point_cloud_from_file(file_path):
    data = np.loadtxt(file_path)

    # Get first 3 indices which are (X, Y, Z)
    points = data[:, :3]

    # Get last 3 indices which are (R, G, B)
    colors = data[:, 3:]

    # Define a point cloud object and fill it with data we have read
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)  # Normalize color values to [0, 1]

    return pcd


def see_point(geometries, point):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere.translate(np.asarray([point.x, point.y, point.z]))
    sphere.paint_uniform_color(np.asarray([1., 0., 0.]))
    sphere.compute_vertex_normals()
    geometries.append(sphere)

    o3d.visualization.draw_geometries(geometries)


def ground_detection(pcd, threshold=0.2):
    pcd_colors = np.asarray(pcd.colors)
    z_min = min(pcd.points, key=lambda x: x[2])[2]

    for i in range(len(pcd.points)):
        if pcd.points[i][2] <= z_min + threshold:
            pcd_colors[i] = [0, 1, 0]

    pcd.colors = o3d.utility.Vector3dVector(pcd_colors)

    return pcd


def main():
    FILE_PATH = "teknofest_ornek_data_v1/ornek_ev_1_lidar_export.xyz"
    pcd = create_point_cloud_from_file(FILE_PATH)

    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

    pcd = ground_detection(pcd)

    geometries = [pcd, origin]
    see_point(geometries, Point(1, 1, 2))
    #o3d.visualization.draw_geometries([pcd], point_show_normal=False)


if __name__ == "__main__":
    main()
