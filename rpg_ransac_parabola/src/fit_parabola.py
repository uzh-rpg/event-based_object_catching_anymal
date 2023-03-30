import numpy as np
from utils import BboxKeys, compute_area_from_bboxes
import cv2
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


GRAVITY = np.array([0, 9.81, 0])  # in positive y direction

camera_matrix = np.array([[592.9264, 0.0, 312.6777],
                          [0.0, 592.4369, 229.7782],
                          [0.000000, 0.000000, 1.000000]])
distortion_coeffs = np.array([-0.3940, 0.2683, 0.0007603, 0.0001349])

def cuboid_data2(o, size=(1, 1, 1)):
    X = [[[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
         [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
         [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
         [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
         [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
         [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]]]
    X = np.array(X).astype(float)
    for i in range(3):
        X[:, :, i] *= size[i]
    X += np.array(o)
    return X


def plotCubeAt2(positions, sizes=None, colors=None, **kwargs):
    if not isinstance(colors, (list, np.ndarray)): colors = ["C0"] * len(positions)
    if not isinstance(sizes, (list, np.ndarray)): sizes = [(1, 1, 1)] * len(positions)
    g = []
    for p, s, c in zip(positions, sizes, colors):
        g.append(cuboid_data2(p, size=s))
    return Poly3DCollection(np.concatenate(g),
                            facecolors=np.repeat(colors, 6), **kwargs)

def bboxes_to_3d_points(bboxes, area_ball=.1**2, camera_matrix=None, distortion_coeffs=None, distortion_model=None):
    # get depth from area
    area = compute_area_from_bboxes(bboxes) * np.pi/4
    area_ratio = (area_ball / area).astype('float')
    depth = np.sqrt(area_ratio) * camera_matrix[0,0]

    # compute 3D points from object center points
    xy = bboxes[:,[BboxKeys.center_x, BboxKeys.center_y]].astype('float')
    xy_norm = cv2.undistortPoints(xy, camera_matrix, distortion_coeffs) # return normalized coords
    xy_norm = xy_norm[:,0]

    xy_norm = np.concatenate([xy_norm, np.ones_like(xy[:,:1])], axis=-1)
    points = xy_norm * depth.reshape((-1,1))

    return points

def minimal_solution_parabola(points_3d, time):
    t0, t1 = time
    p0, p1 = points_3d

    p0_gravity_comp = p0 - GRAVITY * .5 * t0 ** 2
    p1_gravity_comp = p1 - GRAVITY * .5 * t1 ** 2

    v = (p1_gravity_comp - p0_gravity_comp) / (t1 - t0)
    p = p0_gravity_comp - v * t0

    return p, v

def ransac_iter_from_confidence_and_outlier_probability(outlier_probability, confidence=0.99):
    return int(np.log(1-confidence) / np.log(1-(1-outlier_probability)**2))

def least_squares_solution_parabola(points_3d, time):
    # solves a least squares problem
    # error = sum (bx - vx t - px)^2 + (by - 1/2 g t^2 - vy t - py)^2 + (bz - vz t - pz)^2
    # the solution can be separated over vx, px, vy, py and vz, pz
    # 0 = sum bx   - vx sum t   - px n
    # 0 = sum t bx - vx sum t^2 - px sum t
    points_3d_gravity_comp = points_3d - 0.5 * GRAVITY.reshape((1,-1)) * time.reshape((-1,1))**2

    n = len(points_3d)
    tp_sum = (points_3d_gravity_comp * time.reshape((-1,1))).sum(0)
    p_sum = points_3d_gravity_comp.sum(0)
    t_sum = time.sum()
    t_2_sum = (time**2).sum()
    p_opt = (t_sum * tp_sum - p_sum * t_2_sum) / (t_sum**2 - n * t_2_sum)
    v_opt = (t_sum * p_sum - n * tp_sum) / (t_sum**2 - n * t_2_sum)

    return p_opt, v_opt

def compute_inliers(points_3d, time, p, v, inlier_threshold=0.1):
    points_3d_gravity_comp = points_3d - 0.5 * GRAVITY.reshape((1, -1)) * time.reshape((-1, 1)) ** 2
    errors = (points_3d_gravity_comp - v.reshape((1, -1)) * time.reshape((-1, 1)) - p.reshape((1, -1)))
    inliers = (errors**2).sum(-1) < inlier_threshold**2
    return inliers

def reasonable_estimate(p, v):
    #return True
    #impact_point = compute_impact_point(p, v)
    if v[2] > 0:
        return False
    xlims = [ -2,2]
    ylims = [ 0,2]
    return True#(impact_point[0] <= xlims[1] and impact_point[0] >= xlims[0] and impact_point[1] <= ylims[1] and impact_point[1] >= ylims[0])

def adaptive_ransac_parabola_3d(points_3d, time, confidence=0.999, num_max_ransac_iter=100, inlier_threshold=0.1):
    # parabola has equation p(t) = 1/2 g t^2 e_y + v t + p
    # unknowns are p and v
    num_ransac_iter = num_max_ransac_iter
    num_iter_done = 0

    max_num_inliers = 0
    max_p = None
    max_v = None

    while num_iter_done < num_ransac_iter:
        indices = np.random.choice(len(points_3d), size=(2,), replace=False)
        p, v = minimal_solution_parabola(points_3d[indices], time[indices])

        if reasonable_estimate(p, v):
            inliers = compute_inliers(points_3d, time, p, v, inlier_threshold=inlier_threshold)
        else:
            inliers = np.full_like(points_3d[:,0], fill_value=False, dtype="bool")
        num_inliers = inliers.sum()

        if num_inliers > max_num_inliers:
            max_num_inliers = num_inliers
            max_p = p   
            max_v = v

        # estimated number of outliers and estimate num iterations
        estimated_oulier_prob = 1 - num_inliers / len(points_3d)
        # threshold the estimated outlier probability
        if estimated_oulier_prob < 0.000001:
            estimated_oulier_prob = 0.000001
        elif estimated_oulier_prob > 0.99999:
            estimated_oulier_prob = 0.99999
        num_ransac_iter_computed = ransac_iter_from_confidence_and_outlier_probability(estimated_oulier_prob, confidence)
        num_ransac_iter = min([num_ransac_iter_computed, num_max_ransac_iter])

        num_iter_done += 1

    p_opt, v_opt = least_squares_solution_parabola(points_3d[inliers], time[inliers])
    #p_opt, v_opt = max_p, max_v #least_squares_solution_parabola(points_3d[inliers], time[inliers])
    print(f"NUM OF INLIERS {num_inliers}, NUM OF POINTS {points_3d.shape[0]}")
    # print("INLIERS")
    # print(points_3d[inliers])
    # print("FULL ARRAY")
    # print(points_3d)
    # print(time[inliers])
    return p_opt, v_opt, inliers, points_3d

def project_3d_to_camera(points_3d, camera_matrix, distortion_coeffs, distortion_model):
    pixels = np.einsum("ij,nj->ni", camera_matrix, points_3d)
    pixels = pixels[:,:2] / pixels[:,-1:]
    map_xy = np.stack(cv2.initUndistortRectifyMap(camera_matrix, distortion_coeffs, None, camera_matrix, (640, 480), cv2.CV_32FC1), axis=-1)
    x, y = pixels.astype("int32").T
    mask = (x >= 0) & (y >= 0) & (x < 640) & (y < 480)
    pixels_dist = map_xy[y[mask], x[mask]]
    return pixels_dist

def fit_parabola_to_bboxes(points_4d, confidence=0.99, num_max_ransac_iter=100, inlier_threshold=0.1, debug=False, ax=None, handles=None):
    points_4d[:,-1] -= points_4d[0,-1]

    points_3d = points_4d[:,:3]
    init_position, velocity, inliers, points_3d = adaptive_ransac_parabola_3d(points_3d, points_4d[:,-1], confidence, num_max_ransac_iter, inlier_threshold)

    if debug:
        sample_times = np.linspace(0, 1, 1000).reshape((-1,1))
        sampled_points_3d = 0.5 * GRAVITY * sample_times**2 + velocity.reshape((1,-1)) * sample_times + init_position.reshape((1,-1))
        sampled_pixels = project_3d_to_camera(sampled_points_3d, camera_matrix, distortion_coeffs, distortion_model)
        sampled_pixels = sampled_pixels[(sampled_pixels[:,0] >= 0) & (sampled_pixels[:,1] >= 0) & (sampled_pixels[:,1] < 480) & (sampled_pixels[:,0] < 640)]
        debug_image = plot_parabola_in_2d(sampled_pixels, bboxes, inliers)

        # compute impact point, if it is below the net, ignore
        #impact_point = compute_impact_point(init_position, velocity)
        handles = plot_in_3d(sampled_points_3d, bboxes, area_ball, inliers, ax, handles)
        

        return init_position, velocity, debug_image, handles

    return init_position, velocity, inliers, points_3d

def plot_in_3d(sampled_points_3d, bboxes, area_ball, inliers, ax, handles=None):
    # convert to reasonable coodinate system centered at robot. x pointing forward, y pointing right, z pointing up
    #
    x, y, z = sampled_points_3d[:,2], -sampled_points_3d[:,0], -sampled_points_3d[:,1]
    if handles is None:
        handles = ax.plot(x, y, z)
        positions = [(-1, -0.125, 0)]
        sizes = [(1, .25, .25)]
        colors = ["crimson"]
        pc = plotCubeAt2(positions, sizes, colors=colors, edgecolor="k")
        ax.add_collection3d(pc)
        ax.view_init(elev=2, azim=180)
    else:
        handles[0].set_xdata(x)
        handles[0].set_ydata(y)
        handles[0].set_3d_properties(z)
    return handles

def plot_parabola_in_2d(pixels, bboxes, inliers):
    width, height = 640, 480
    image_panel = np.zeros((4 * height, 4 * width, 3))
    for index, bbox in enumerate(reversed(bboxes)):
        col = index % 4
        row = index // 4
        image_idx = int(bbox[BboxKeys.image_idx])
        image = cv2.imread("thresh/thresh_image_%s.png" % image_idx)
        font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
        t = round(bbox[BboxKeys.time], 3)
        frame = cv2.putText(image, f"t = {t} s", (10, 60), font, 1, (210, 155, 155), 4, cv2.LINE_8)

        channel = 1 if inliers[index] > 0 else 2
        image_panel[height * row:height * row + height, width * col:width * col + width, channel] = image[...,0]
        shifted_pixels = pixels.astype("int32")
        shifted_pixels[:,0] += 640 * col
        shifted_pixels[:,1] += 480 * row
        image_panel = cv2.polylines(image_panel, [shifted_pixels], False, (255, 50, 50), 1)

    image_panel = cv2.resize(image_panel, (640, 480))
    return image_panel

def sample_parabola(t, p, v):
    return .5 * GRAVITY.reshape((1,3)) * t.reshape((-1,1))**2 + v.reshape((1,3)) * t.reshape((-1,1)) + p.reshape((1,3))

def compute_impact_point(initial_position, initial_velocity):
    # solves equation p_impact = 1/2 g t^2 + v t + p, where p_impact_z = 0
    # this results in t = -p_z / v_z
    px, py, pz = initial_position
    vx, vy, vz = initial_velocity

    t_impact = - pz / vz

    impact_position_x = vx * t_impact + px
    impact_position_y = 0.5 * GRAVITY[1] * t_impact**2 + vy * t_impact + py

    return np.array([impact_position_x, impact_position_y, 0])
