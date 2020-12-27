import numpy as np
import open3d as o3d
import copy

def draw_registration_result(source, target, transformation=np.identity(4)):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

if __name__ == '__main__':
    print('Trying to align large surfaces')

    filename1 = '/Users/michaelko/Downloads/wetransfer-845f7e/15dec2020_horisontalRulerSection11.ply'
    filename2 = '/Users/michaelko/Downloads/wetransfer-845f7e/15dec2020_horisontalRulerSection21.ply'

    pc1 = o3d.io.read_point_cloud(filename1)
    pc2 = o3d.io.read_point_cloud(filename2)

    pc1.paint_uniform_color([0.9, 0.7, 0])
    pc2.paint_uniform_color([0, 0.651, 0.929])

    # Find 

    reg_p2p = o3d.pipelines.registration.registration_icp(pc1, pc2, 0.02)

    draw_registration_result(pc1, pc2, reg_p2p.transformation)
    # o3d.visualization.draw_geometries([pc1, pc2])

    print('Read meshes')

