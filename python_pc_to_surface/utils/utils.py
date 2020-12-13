import copy
import numpy as np
import open3d as o3d


# Reads point_cloud.txt in format xyzn
def read_pcd(path: str) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(filename=path, format="xyzn", print_progress=True)
    pcd = pcd.paint_uniform_color(color=np.array([0.71, 0.79, 0.957]))
    print("Try to render a point cloud with normals (exist: " +
          str(pcd.has_normals()) + ") and colors (exist: " +
          str(pcd.has_colors()) + ")")
    return pcd


# Reads math_model.obj
def read_obj(path: str) -> o3d.geometry.TriangleMesh:
    mesh = o3d.io.read_triangle_mesh(filename=path, print_progress=True)
    mesh = mesh.paint_uniform_color(np.array([1, 0.706, 0.929]))
    print("Try to render a mesh with normals (exist: " +
          str(mesh.has_vertex_normals()) + ") and colors (exist: " +
          str(mesh.has_vertex_colors()) + ")")
    return mesh


# Changes comma-delimiter to space-delimiter
def commas_to_spaces():
    import re
    input_path, output_path = "data/", "point_clouds/"
    fn = input('Enter file name: ')

    lines = []
    try:
        file = open(input_path + fn, 'r')
        for line in file:
            lines.append(re.sub(',+', ' ', line))
        file.close()
    except FileNotFoundError:
        print(f"File {fn} not found in data\\")

    res = open(output_path + fn, 'w')
    res.writelines(lines)
    res.close()
    print(f'PointCloud.txt updated and located in "..\\point_clouds\\{fn}"')


# Connects objects to one object
def correct_obj():
    import re
    input_path, output_path = "models/", "models/corrected"
    fn = input("Enter file name: ")

    mtl, vs, vts, vns, fs = [], [], [], [], []
    try:
        file = open(input_path + fn, 'r')
        for _ in range(4):
            mtl.append(file.readline())

        for line in file:
            s = line.strip().split(' ')[0]
            if s == "v":
                vs.append(line)
            elif s == "vt":
                vts.append(line)
            elif s == "vn":
                vns.append(line)
            elif s == "f":
                fs.append(line)
            else:
                continue
        file.close()
    except FileNotFoundError:
        print(f"File {fn} not found in data\\")

    res = open(output_path + fn, 'w')
    res.writelines(mtl)
    res.writelines(vs)
    res.writelines(vts)
    res.writelines(vns)
    res.writelines(["usemtl c_0", "s 1"])
    res.writelines(fs)
    res.close()
    print(f'ObjectFile updated and located in "..\\models\\corrected\\{fn}"')


# Downsample point cloud and compute it's FPFH feature
def preprocess_point_cloud(pcd: o3d.geometry.PointCloud, voxel_size: np.float64) -> {
    o3d.geometry.PointCloud, o3d.pipelines.registration.Feature
}:
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    # KDTreeSearchParamHybrid: radius - search radius, max_nn - at maximum,
    # max_nn neighbours will be searched
    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f.\n" % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    # print(pcd_fpfh, '\n')
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size: np.float64) -> {
    o3d.geometry.PointCloud, o3d.geometry.PointCloud, o3d.geometry.PointCloud,
    o3d.geometry.PointCloud, o3d.pipelines.registration.Feature,
    o3d.pipelines.registration.Feature, o3d.geometry.TriangleMesh
}:
    print(":: Load two point clouds and disturb initial pose.")
    source_s = o3d.io.read_triangle_mesh("models/cube.obj")
    source = source_s.sample_points_uniformly(number_of_points=50000)
    target = o3d.io.read_point_cloud('point_clouds/cube.txt', format='xyzn')
    threshold = 0.02
    trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5], [-0.139, 0.967, -0.215, 0.7],
                             [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
    # source.transform(trans_init)
    draw_registration_result(source, target, trans_init)  # np.identity(4)
    print("1. Initial alignment.")
    evaluation = o3d.pipelines.registration.evaluate_registration(  # it's RegistrationResult
        source, target, threshold, trans_init
    )
    print(evaluation)
    print(evaluation.transformation)

    print("\n2. Computing Features (Fast Point Feature Histogram).")
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh, source_s


def draw_registration_result(src: o3d.geometry.TriangleMesh,
                             trg: o3d.geometry.PointCloud,
                             transformation) -> None:
    src_tmp = copy.deepcopy(src)
    trg_tmp = copy.deepcopy(trg)
    src_tmp.paint_uniform_color([1, 0.706, 0])
    trg_tmp.paint_uniform_color([0, 0.651, 0.929])
    src_tmp.transform(transformation)
    o3d.visualization.draw_geometries([src_tmp, trg_tmp])
                                      # zoom=0.4559,
                                      # front=[0.6452, -0.3036, -0.7011],
                                      # lookat=[1.9892, 2.0208, 1.8945],
                                      # up=[-0.2779, -0.9482, 0.1556],

