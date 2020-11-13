import numpy as np
import open3d as o3d

pcd1 = o3d.io.read_point_cloud('cube3.txt', format='xyzn')
print(pcd1)

print("Testing IO for meshes ...")
mesh = o3d.io.read_triangle_mesh("cube.obj")
print(mesh)



mesh.paint_uniform_color([1, 0.706, 0])
print("Try to render a mesh with normals (exist: " +
      str(mesh.has_vertex_normals()) + ") and colors (exist: " +
      str(mesh.has_vertex_colors()) + ")")

pcd = mesh.sample_points_uniformly(number_of_points=50000)

cent =pcd.get_center()   #model



pcd1 = pcd1.voxel_down_sample(1)
#pcd1 = pcd1.uniform_down_sample(10)
cent2 = pcd1.get_center()  # cloud
cent2 -= cent



#mesh.translate(pcd1.get_center())

mesh.translate(cent2)


cent1 = mesh.get_center()
print(cent1)
print(cent2)

c1 = o3d.geometry.PointCloud()
c1.points = o3d.utility.Vector3dVector(np.array([cent1,cent2]))

o3d.visualization.draw_geometries([c1,mesh,pcd1])

# f 2/21/21 1/22/22 7/23/23
# f 7/23/23 1/22/22 5/24/24


