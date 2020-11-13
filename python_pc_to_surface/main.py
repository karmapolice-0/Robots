import numpy as np
import open3d as o3d

input_path = "input_data/"
output_path = "output_data/"
dataname = input('Enter file name: ')
point_cloud = np.loadtxt(output_path + dataname, skiprows=0)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])
# pcd.colors = o3d.utility.Vector3dVector(point_cloud[:, 3:6] / 255)
pcd.normals = o3d.utility.Vector3dVector(point_cloud[:, 3:6])
o3d.visualization.draw_geometries([pcd])

# ========= Ball-Pivoting Algorithm

# radius deter
'''distances = pcd.compute_nearest_neighbor_distance()
avg_distance = np.mean(distances)
radius = 3 * avg_distance

# computing the mesh
bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd,
                                                                           o3d.utility.DoubleVector(
                                                                               [radius, radius * 2]))
# decimating the mesh
dec_mesh = bpa_mesh.simplify_quadric_decimation(100000)
dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()

o3d.visualization.draw_geometries([dec_mesh])
'''
#poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
#    pcd=pcd, depth=8, width=0, scale=1.1, linear_fit=False)[0]
#bbox = pcd.get_axis_aligned_bounding_box()
#p_mesh_crop = poisson_mesh.crop(bbox)
#o3d.visualization.draw_geometries([poisson_mesh])

