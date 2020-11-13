import numpy as np
import open3d as o3d


def read_pcd(path: str) -> o3d.geometry.PointCloud:
    filename = input('Enter file name: ')
    pcd = o3d.io.read_point_cloud(path+filename, format="xyzn")
    pcd = pcd.paint_uniform_color(color=np.array([0.71, 0.79, 0.957]))
    pcd = down_sample(pcd, np.float64(2.5))
    print("Try to render a point cloud with normals (exist: " +
          str(pcd.has_normals()) + ") and colors (exist: " +
          str(pcd.has_colors()) + ")")
    return pcd


def read_obj(path: str) -> o3d.geometry.TriangleMesh:
    filename = input("Enter obj file: ")
    mesh = o3d.io.read_triangle_mesh(path + filename)
    mesh = mesh.paint_uniform_color(np.array([1, 0.706, 0]))
    print("Try to render a mesh with normals (exist: " +
          str(mesh.has_vertex_normals()) + ") and colors (exist: " +
          str(mesh.has_vertex_colors()) + ")")
    return mesh


def down_sample(pcd: o3d.geometry.PointCloud, param: np.float64) -> o3d.geometry.PointCloud:
    return pcd.voxel_down_sample(param)
