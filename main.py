import open3d as o3d
import numpy as np


class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def create_point_cloud_from_list(point_cloud):
    points = [[point.x, point.y, point.z] for point in point_cloud]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd


def read_point_cloud_data(file_path):
    point_cloud = []
    with open(file_path, 'r') as file:
        for line in file:
            components = line.split()
            x = float(components[0])
            y = float(components[1])
            z = float(components[2])
            point_cloud.append(Point(x, y, z))
    return point_cloud


def downsampled_pcd(pcd):
    return pcd.voxel_down_sample(voxel_size=0.05)


def vertex_normal_estimation(pcd):
    return pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))


def see_point(geometries, point):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
    sphere.translate(np.asarray([point.x, point.y, point.z]))
    sphere.paint_uniform_color(np.asarray([1., 0., 0.]))
    sphere.compute_vertex_normals()
    geometries.append(sphere)

    o3d.visualization.draw_geometries(geometries)


def main():
    FILE_PATH = "teknofest_ornek_data_v1/ornek_ev_1_lidar_export.xyz"

    point_cloud_data = read_point_cloud_data(FILE_PATH)
    pcd = create_point_cloud_from_list(point_cloud_data)

    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    geometries = [pcd, origin]

    see_point(geometries, Point(1, 1, 2))

    #o3d.visualization.draw_geometries([pcd], point_show_normal=False)


if __name__ == "__main__":
    main()
