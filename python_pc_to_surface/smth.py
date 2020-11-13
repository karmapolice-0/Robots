import open3d as o3d
from utils.read_pcd import read_pcd


input_path = "output_data/"
pcd = read_pcd(input_path)
center = pcd.get_center()
print(center)

o3d.visualization.draw_geometries([pcd],
                                  point_show_normal=False,
                                  mesh_show_back_face=False)
