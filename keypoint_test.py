import numpy as np
import open3d as o3d
import scipy

class Storage2D:
    """
    The file represents the point cloud as a 2D hash table
    """
    def __init__(self):
        pass


def perpendicular_vector(v):
    # return vector perpendicular to the given one
    if v[0] == 0 and v[1] == 0:
        if v[2]:
            # v is Vector(0, 0, 0)
            raise ValueError('zero vector')

        # v is Vector(0, 0, v.z)
        return np.array([0, 1, 0])

    val = np.sqrt(v[0]*v[0] + v[1]*v[1])
    return np.array([-v[1], v[0], 0]) / val

def hom_lin_eq(A):
    u, s, vh = np.linalg.svd(A)
    return vh[:, -1]


# Point cloud preprocessing
def preprocess(pcd, n_random_tests=500, n_points=150, median_rad_coef=5, filename=''):
    """
    Given a point cloud return the filtered point cloud. Specifically - remove outliers and apply a low pass filter
    :param pcd: the point cloud
    :params n_random_tests: the number of random samples we are doing to estimate the median 150 point radius
    :param n_points: the number of points in the radius test
    :param median_rad_coef: we multiply the median radius by that number and define as outliers all points that do not
        who has fewer than n_points whithin that radius
    :return: updated point cloud
    """
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)

    # Compute for every point the bounding sphere of its 100 neighbors. Remove all the points for which
    # the bounding sphere is much larger or which have much fewer than 100 neighbors in the typical bounding sphere


    # Go over random points to compute the typical bounding sphere of n points
    rad_arr = np.zeros(n_random_tests)
    for k in range(n_random_tests):
        pt_ind = np.random.randint(0, np.asarray(pcd.points).shape[0])
        [_, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[pt_ind], n_points)
        # Compute the bounding sphere
        center = np.mean(np.asarray(pcd.points)[idx], axis=0)
        rad = np.asarray(pcd.points)[idx] - center
        rad_arr[k] = np.max(np.sum(rad*rad, axis=1))

    median_rad = np.sort(rad_arr)[int(n_random_tests/2)]

    # TODO: This function removes also points at corners and sides
    (pcd, _) = pcd.remove_radius_outlier(n_points, median_rad * median_rad_coef)
    pass

    # for k in range(np.asarray(pcd.points).shape[0]):
    #
    #
    #     pcd_tree.search_radius_vector_3d(pcd.points[k], n_points)


    # Fix normals
    pcd.normalize_normals()
    pcd.orient_normals_towards_camera_location()
    # Fix point cloud
    # remove_radius_outlier(self, nb_points, radius)

    if filename != '':
        o3d.io.write_point_cloud(filename, pcd)

    return pcd


def compute_mesh(pcd, depth=9, mesh_name=''):
    """
    The function create a triangulated mesh from the point cloud. The reason is to make subsequent computations faster
    and more accurate
    If the mesh vertices have 1 to 1 correspondence with point cloud vertices, going back is trivial
    :param pcd: - point cloud
    :param mesh_name: if not equal to '', the mesh is saved
    :return:
    """

    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)

    # Smoothing
    # mesh = mesh.filter_smooth_simple(number_of_iterations=20)

    mesh.remove_non_manifold_edges()
    nonmanifold = mesh.get_non_manifold_vertices()
    iter = 0
    while len(nonmanifold) > 0:
        mesh.remove_vertices_by_index(nonmanifold)
        nonmanifold = mesh.get_non_manifold_vertices()
        iter = iter + 1
        if iter > 4:
            break



    mesh.compute_vertex_normals()

    if mesh_name != '':
        o3d.io.write_triangle_mesh(mesh_name, mesh)
        print('Done saving ', mesh_name)

    return mesh

def basic_test(pcd):
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)

    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    [_, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[50000], 100)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

    # Fix normals
    np.asarray(pcd.normals)[np.where(np.asarray(pcd.normals)[:, 2] < 0), :] = \
        -np.asarray(pcd.normals)[np.where(np.asarray(pcd.normals)[:, 2] < 0), :]

    n_points = 90
    bad_pts = 0
    curv_arr = np.zeros((np.asarray(pcd.points).shape[0], 2))
    for k in range(np.asarray(pcd.points).shape[0]):
        # for every point get the 50 nearest neighbors and estimate the curvature
        # The curvature is simply the second derivative
        # z = a0x^2 + a1y^2 + a2*x*y + a3*x + a4*y + a5
        nv = np.asarray(pcd.normals)[k, :]
        xv = perpendicular_vector(nv)
        yv = np.cross(nv, xv)
        [_, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[k], n_points)
        nns = np.asarray(pcd.points)[idx, :] - pcd.points[k]
        xcoor = np.matmul(nns, xv)
        ycoor = np.matmul(nns, yv)
        zcoor = np.matmul(nns, nv)
        A = np.zeros((n_points, 3))
        A[:, 0] = xcoor * xcoor
        A[:, 1] = ycoor * ycoor
        A[:, 2] = xcoor * ycoor
        # A[:, 3] = xcoor
        # A[:, 4] = ycoor
        B = np.matmul(A.T, zcoor)
        AA = np.matmul(A.T, A)
        _, d, _ = np.linalg.svd(AA)
        d = np.log10(d)
        if d[2] > -12:
            res = np.linalg.solve(AA, B)
        else:
            bad_pts = bad_pts + 1
            res = np.zeros(3)
            #
        # res = hom_lin_eq(AA)
        D = np.array([[res[0], res[2]], [res[2], res[1]]])
        _, d, _ = np.linalg.svd(D)
        curv_arr[k, 0] = d[0] * np.sign(res[0])
        curv_arr[k, 1] = d[1] * np.sign(res[1])
        if k % 500 == 0:
            print(k)
            print(bad_pts)

    # Change point colors
    min_curv = np.percentile(curv_arr, 1, axis=0)
    max_curv = np.percentile(curv_arr, 99, axis=0)
    pass
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    np.asarray(pcd.colors)[:, 0] = (curv_arr[:, 0] - min_curv[0]) / (max_curv[0] - min_curv[0])
    np.asarray(pcd.colors)[:, 1] = (curv_arr[:, 1] - min_curv[1]) / (max_curv[1] - min_curv[1])
    o3d.io.write_point_cloud('/Users/michaelko/Downloads/wetransfer-845f7e/small_color.ply', pcd)


if __name__ == '__main__':
    print('Compute keypoints on a surface')

    # filename = '/Users/michaelko/Downloads/wetransfer-845f7e/21_small.ply'
    filename = '/Users/michaelko/Downloads/wetransfer-845f7e/small_5.ply'
    pcd = o3d.io.read_point_cloud(filename)

    # preprocessing is not neeed for small_5 mesh
    # pcd = preprocess(pcd, filename='/Users/michaelko/Downloads/wetransfer-845f7e/small_5.ply')
    compute_mesh(pcd, depth=9, mesh_name='/Users/michaelko/Downloads/wetransfer-845f7e/mesh1.ply')
    # compute_mesh(pcd, depth=11, mesh_name='/Users/michaelko/Downloads/wetransfer-845f7e/mesh2.ply')
    # compute_mesh(pcd, depth=7, mesh_name='/Users/michaelko/Downloads/wetransfer-845f7e/mesh3.ply')



