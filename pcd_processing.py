import open3d as o3d
import numpy as np
from sklearn.preprocessing import StandardScaler
import skimage.color as scolor


def perpendicular_vector(v):
    # return vector perpendicular to the given one
    if v[0] == 0 and v[1] == 0:
        if v[2] == 0:
            # v is Vector(0, 0, 0)
            raise ValueError('zero vector')

        # v is Vector(0, 0, v.z)
        return np.array([0, 1, 0])

    val = np.sqrt(v[0]*v[0] + v[1]*v[1])
    return np.array([-v[1], v[0], 0]) / val


class PCDProcessor:
    """
    The class provides various computations not existing in open3d
    """
    def __init__(self, pcd=None):
        self.init = False
        # The KDD tree representing the point cloud
        self.pcd_tree = None
        # The distance (in units) representing the finest scale at which we can work
        self.median_radius = -1
        self.n_fine_scale_points = 150
        if pcd is not None:
            self.init_from_pcd(pcd)

    def init_from_pcd(self, pcd):
        self.pcd_tree = o3d.geometry.KDTreeFlann(pcd)

        # Run ranom tests to define median radius
        n_random_tests = 500


        rad_arr = np.zeros(n_random_tests)
        for k in range(n_random_tests):
            pt_ind = np.random.randint(0, np.asarray(pcd.points).shape[0])
            [_, idx, _] = self.pcd_tree.search_knn_vector_3d(pcd.points[pt_ind], self.n_fine_scale_points)
            # Compute the bounding sphere
            center = np.mean(np.asarray(pcd.points)[idx], axis=0)
            rad = np.asarray(pcd.points)[idx] - center
            rad_arr[k] = np.max(np.sum(rad * rad, axis=1))

        self.median_radius = np.median(rad_arr)

    def clean_pcd(self, pcd):
        """
        Remove bad vertices from pcd
        :return:
        """
        (pcd, _) = pcd.remove_radius_outlier(self.n_fine_scale_points, self.median_radius * 5)
        return pcd

    def set_normals(self, pcd):
        """
        The function computes normals and orients them accordingly
        :param pcd: the input point cloud
        :return: point cloud with normals
        """
        # For every point, estimate the PCA and take the smallest eignevector
        num_pts_arr = np.zeros((np.asarray(pcd.points).shape[0], 1))
        for k in range(np.asarray(pcd.points).shape[0]):
            [_, idx, _] = self.pcd_tree.search_radius_vector_3d(pcd.points[k], self.median_radius * 2)
            num_pts_arr[k, 0] = len(idx)
            if len(idx) < 4:
                pcd.normals[k] = np.asarray([0, 0, -1])
                continue
            scaled_pts = StandardScaler().fit_transform(np.asarray(pcd.points)[idx, :])
            cov_matrix = np.cov(scaled_pts.T)
            values, vectors = np.linalg.eig(cov_matrix)
            if vectors[np.argmin(values)][2] < 0:
                pcd.normals[k] = vectors[np.argmin(values)]
            else:
                pcd.normals[k] = -vectors[np.argmin(values)]

        return pcd

    def compute_pcd_curvature(self, pcd, method='polynomaial'):
        """
        Compute the curvature of the input point cloud
        :param pcd:
        :return:
        """
        # Gauss - K, H
        # k1 = H - sqrt(H^2 - K)
        # k2 = H + sqrt(H^2 - K)
        # https://core.ac.uk/download/pdf/82403948.pdf
        # H, K
        curv_arr = np.zeros((np.asarray(pcd.points).shape[0], 2))
        for k in range(np.asarray(pcd.points).shape[0]):
            [_, idx, _] = self.pcd_tree.search_radius_vector_3d(pcd.points[k], self.median_radius * 2)
            pts = np.asarray(pcd.points)[idx, :] - pcd.points[k]
            nv = pcd.normals[k]
            xv = perpendicular_vector(nv)
            yv = np.cross(nv, xv)
            xcoor = np.matmul(pts, xv)
            ycoor = np.matmul(pts, yv)
            zcoor = np.matmul(pts, nv)
            # r = a0x^2 + a1y^2 + a2*x*y + a3*x + a4*y + a5
            A = np.zeros((len(idx), 3))
            A[:, 0] = xcoor * xcoor
            A[:, 1] = ycoor * ycoor
            A[:, 2] = xcoor * ycoor

            B = np.matmul(A.T, zcoor)
            AA = np.matmul(A.T, A)

            _, d, _ = np.linalg.svd(AA)
            d = np.log10(d)
            if d[2] > -12:
                res = np.linalg.solve(AA, B)
            else:
                res = np.zeros(3)
            D = np.array([[res[0], res[2]], [res[2], res[1]]])
            _, d, _ = np.linalg.svd(D)
            curv_arr[k, 0] = d[0] * np.sign(res[0])
            curv_arr[k, 1] = d[1] * np.sign(res[1])

        return curv_arr


if __name__ == '__main__':
    print('Example of PCD processing')
    # The mesh is preprocessed already
    filename = '/Users/michaelko/Downloads/wetransfer-845f7e/small_5.ply'
    pcd = o3d.io.read_point_cloud(filename)
    processor = PCDProcessor(pcd)
    pcd = processor.set_normals(pcd)
    curv = processor.compute_pcd_curvature(pcd)

    # Set colors according to the curvature
    # Use the curvature to set HUE
    colors_hsv = np.zeros((np.asarray(pcd.points).shape[0], 3))
    colors_hsv[:, 1] = 0.9
    colors_hsv[:, 2] = 0.9
    mean_curv = curv[:, 0] + curv[:, 1]
    max_curv = np.max(mean_curv) # 120
    min_curv = np.min(mean_curv) # 0
    colors_hsv[:, 0] = 0.66 * (mean_curv - min_curv) / (max_curv - min_curv)

    pcd.paint_uniform_color([0.9, 0.7, 0])
    rgb_colors = scolor.hsv2rgb(colors_hsv)
    for k in range(np.asarray(pcd.points).shape[0]):
        pcd.colors[k] = rgb_colors[k, :]
    o3d.visualization.draw_geometries([pcd])
