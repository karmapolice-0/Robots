import numpy as np
import open3d as o3d


def read_pcd(path: str) -> o3d.geometry.PointCloud:
    filename = input('Enter file name: ')
    point_cloud = np.loadtxt(path + filename)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
    pcd.normals = o3d.utility.Vector3dVector(point_cloud[:, 3:6])

    clr = np.array([0.71, 0.79, 0.957])
    pcd = pcd.paint_uniform_color(color=clr)
    pcd = pcd.voxel_down_sample(2.5)
    return pcd


def down_sample(pcd: o3d.geometry.PointCloud, param: np.float64) -> o3d.geometry.PointCloud:
    return pcd.voxel_down_sample(param)
