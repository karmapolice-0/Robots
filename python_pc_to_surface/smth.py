import open3d as o3d
from utils.utils import read_pcd, read_obj


pcd_path, obj_path = "point_clouds/", "models/"
pcd = read_pcd(pcd_path)
obj = read_obj(obj_path)
pcd_center, obj_center = pcd.get_center(), obj.get_center()
print(pcd_center, "\n", obj_center)


o3d.visualization.draw_geometries([pcd, obj],
                                  point_show_normal=False,
                                  mesh_show_back_face=False)
