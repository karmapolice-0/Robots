import numpy as np
import open3d as o3d


def read_pcl(path):
    filename = input('Enter file name: ')
    point_cloud = np.loadtxt(path + filename)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
    pcd.normals = o3d.utility.Vector3dVector(point_cloud[:, 3:6])
    clr = np.array([0.71, 0.79, 0.957])
    pcd = pcd.paint_uniform_color(color=clr)
    pcd = pcd.voxel_down_sample(2.5)
    return pcd

print("Testing IO for meshes ...")
mesh = o3d.io.read_triangle_mesh("cube.obj")
print(mesh)
cent = []
cent = mesh.get_center()
print (cent)

mesh.paint_uniform_color([1, 0.706, 0])
print("Try to render a mesh with normals (exist: " +
      str(mesh.has_vertex_normals()) + ") and colors (exist: " +
      str(mesh.has_vertex_colors()) + ")")


pcd = mesh.sample_points_uniformly(number_of_points=5000)


# Re = np.array([
#      [1,1,1,1],
#      [1,1,1,1],
#      [1,1,1,1],
#      [-276.61238929, 856.52305743, -3.35950278]
#      #[1,1,1,1]
#               ])


r = np.array([-276.61238929, 856.52305743, -3.35950278])

c = ([],[],[])
c = mesh.get_rotation_matrix_from_axis_angle(cent)
c = c*2
#mesh.translate(r)
#mesh.transform(c)
# c = o3d.cpu.pybind.geometry.Geometry3D
# c = mesh.rotate(R1, R, cent)
mesh.rotate(c,c,cent)
#o3d.visualization.draw_geometries([pcd, mesh])

