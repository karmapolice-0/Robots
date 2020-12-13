import numpy as np
import open3d as o3d
import time
from utils.utils import prepare_dataset, draw_registration_result


def ransac_registration(source_down: o3d.geometry.PointCloud, target_down: o3d.geometry.PointCloud,
                        source_fpfh: o3d.pipelines.registration.Feature,
                        target_fpfh: o3d.pipelines.registration.Feature, voxel_size: np.float64) -> \
        o3d.pipelines.registration.RegistrationResult:
    '''
    RANSACConvergenceCriteria - определяет критерий сходимости. RANSAC останавливается
        если кол-во итераций достигает max_iteration, или проверка прошла max_validation
        раз. Проверка - самая вычислительно затратная операция; важна для времени работы
        алгоритма.

    '''
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        20, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                2),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                voxel_size)  # (distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))  # (4000000, 500)
    return result


def fast_ransac_registration(source_down: o3d.geometry.PointCloud,
                             target_down: o3d.geometry.PointCloud,
                             source_fpfh: o3d.pipelines.registration.Feature,
                             target_fpfh: o3d.pipelines.registration.Feature,
                             voxel_size: np.float64) -> \
        o3d.pipelines.registration.RegistrationResult:
    distance_threshold = voxel_size * 1.5
    print(":: Apply fast global registration with distance threshold %.3f" \
          % distance_threshold)
    result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold
        )
    )
    return result


def icp_point_to_point_registration(source, target, voxel_size, result_ransac):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


def demo_ransac_registration():
    print("Demo for manual RANSAC")
    voxel_size = np.float64(0.2)
    # отрисовка 1
    source, target, source_down, target_down, source_fpfh, target_fpfh, source_s = prepare_dataset(voxel_size)
    source.paint_uniform_color([1, 0.706, 0])
    target.paint_uniform_color([0, 0.651, 0.929])

    start = time.time()
    # result_fast = fast_ransac_registration(source_down, target_down,
    #                                        source_fpfh, target_fpfh,
    #                                        voxel_size)
    # print("Fast global registration took %.3f sec.\n" % (time.time() - start))
    # print(result_fast)
    # draw_registration_result(source_down, target_down, result_fast.transformation)

    print("3. RANSAC registration.")
    result_ransac = ransac_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
    print("::RANSAC registration took %.3f sec.\n" % (time.time() - start))
    print(result_ransac, '\n')
    # отрисовка 2
    draw_registration_result(source_down, target_down, result_ransac.transformation)

    result_icp = icp_point_to_point_registration(source, target, voxel_size, result_ransac)
    print(result_icp)
    # отрисовка 3
    draw_registration_result(source_s, target, result_icp.transformation)


if __name__ == "__main__":
    demo_ransac_registration()
