import numpy as np
import open3d as o3d
import copy
import time

def draw_registration_result(source, target, transformation=np.identity(4)):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    # o3d.visualization.draw_geometries([source_temp, target_temp],
    #                                   zoom=0.4459,
    #                                   front=[0.9288, -0.2951, -0.2242],
    #                                   lookat=[1.6784, 2.0612, 1.4451],
    #                                   up=[-0.3402, -0.9189, -0.1996])
    o3d.visualization.draw_geometries([source_temp, target_temp])


if __name__ == '__main__':
    print('Trying to align large surfaces')

    file_list = []
    file_list.append('/Users/michaelko/Downloads/wetransfer-845f7e/15dec2020_horisontalRulerSection11.ply')
    file_list.append('/Users/michaelko/Downloads/wetransfer-845f7e/15dec2020_horisontalRulerSection21.ply')
    file_list.append('/Users/michaelko/Downloads/wetransfer-845f7e/15dec2020_horisontalRulerSection31.ply')
    file_list.append('/Users/michaelko/Downloads/wetransfer-845f7e/15dec2020_horisontalRulerSection41.ply')

    for k in range(3):
        pc1 = o3d.io.read_point_cloud(file_list[k])
        for m in range(k, 4):
            pc2 = o3d.io.read_point_cloud(file_list[m])

            start = time.time()
            reg_p2p = o3d.pipelines.registration.registration_icp(pc1, pc2, max_correspondence_distance=2,
                                    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())
            end = time.time()
            print('Transfomration ', k, ' ', m)
            print(reg_p2p.transformation)
            print('Time ', end - start)

    #
    #
    # # pc1.estimate_normals()
    # # pc2.estimate_normals()
    # #
    # # o3d.io.write_point_cloud('/Users/michaelko/Downloads/wetransfer-845f7e/15dec2020_horisontalRulerSection31.ply', pc1)
    # # o3d.io.write_point_cloud('/Users/michaelko/Downloads/wetransfer-845f7e/15dec2020_horisontalRulerSection41.ply', pc2)
    #
    # pc1.paint_uniform_color([0.9, 0.7, 0])
    # pc2.paint_uniform_color([0, 0.651, 0.929])
    #
    # # Find bounding box of the ruler - it will help me to estimate the ICP threshold
    # # boundingBox = o3d.geometry.AxisAlignedBoundingBox()
    # # boundingBox = boundingBox.create_from_points(pc1.points)
    #
    # draw_registration_result(pc1, pc2)
    #
    #
    #
    # print(reg_p2p.transformation)
    # draw_registration_result(pc1, pc2, reg_p2p.transformation)
    # # o3d.visualization.draw_geometries([pc1, pc2])

    print('Read meshes')

