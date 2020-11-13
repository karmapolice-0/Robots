import numpy as np
from shapely import geometry


def spatial_median(point_cloud, radius):
    new_z = []
    i = 0

    for point in point_cloud:
        point = geometry.Point(point_cloud[i, :])
        patch = geometry.MultiPoint(list(point_cloud[
                                             (point_cloud[:, 0] > point.x - radius + 0.5) &
                                             (point_cloud[:, 0] < point.x + radius + 0.5) &
                                             (point_cloud[:, 1] > point.y - radius + 0.5) &
                                             (point_cloud[:, 1] < point.y + radius + 0.5)
                                             ]))
        pbuff = point.buffer(radius)

        isect = pbuff.intersection(patch)

        plist = []

        if isect.geom_type == 'MultiPoint':
            for p in isect:
                plist.append(p.z)
            new_z.append(np.median(plist))
        else:
            new_z.append(isect.z)

    i += 1
    return new_z


input_path, output_path = "output_data/", "output_data/"
fn = input('Enter file name: ')

lines = []
try:
    file = open(input_path+fn, 'r')
    file1 = open(output_path + fn + ' (прореженный)', 'w')

    pc = np.loadtxt(output_path + fn, skiprows=0)
    file.close()

    res = list(spatial_median(pc, 0.5))
    file1.writelines(res)
    file1.close()
except FileNotFoundError:
    print(f"File {fn} not found")

print(f'Point Cloud txt updated and located in "../output_data/{fn}"')
