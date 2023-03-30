import numpy as np

import cv2
import matplotlib.pyplot as plt
from utils import load_bboxes, filter_to_max_bboxes, BboxKeys, filter_bboxes_to_interval
from fit_parabola import fit_parabola_to_bboxes, compute_impact_point
import time


camera_matrix = np.array([[592.9264, 0.0, 312.6777],
                          [0.0, 592.4369, 229.7782],
                          [0.000000, 0.000000, 1.000000]])
distortion_coeffs = np.array([-0.3940, 0.2683, 0.0007603, 0.0001349])


bboxes_buffer = []

# rolling buffer of bboxes to consider for parabola fitting
bboxes_buffer_len = 5

# discard bboxes older than 0.5 secs from buffer
time_window_oldest = 0.5

# RANSAC discards bboxes for parabola fitting, which are 0.2m away from the parabola
inlier_threshold=0.2
bboxes = load_bboxes("detection.txt")

# start time for visualization
t = 21.5

t_max = bboxes[-1,BboxKeys.time]

t_step = 0.01

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_ylim([-1, 1])
ax.set_xlim([0, 2])
ax.set_zlim([0, 2])
ax.set_ylabel("Y - [m]")
ax.set_zlabel("Z - [m]")
ax.set_xlabel("X - [m]")
handles = None

while t < t_max:

    plt.show(block=False)
    fig.canvas.draw()
    plt.pause(0.001)

    t += t_step
    start = time.time()
    # add new detections, if they appear
    new_bboxes = bboxes[(bboxes[:,BboxKeys.time] < t) & (bboxes[:,BboxKeys.time] > t - t_step)].copy()
    if len(new_bboxes) == 0:
        continue

    bboxes_buffer.append(new_bboxes)

    if len(bboxes_buffer) > bboxes_buffer_len:
        bboxes_buffer.pop(0)

    # discard old samples
    while bboxes_buffer[0][0,BboxKeys.time] < t - time_window_oldest:
        bboxes_buffer.pop(0)

    if len(bboxes_buffer) < 2:
        continue

    
    
    bboxes_to_process = np.concatenate(bboxes_buffer, axis=0)
    max_bboxes = filter_to_max_bboxes(bboxes_to_process)

    initial_position, initial_velocity, debug_image, handles = fit_parabola_to_bboxes(max_bboxes, camera_matrix=camera_matrix,
                                                                                      distortion_coeffs=distortion_coeffs,
                                                                                      inlier_threshold=inlier_threshold,
                                                                                      debug=True, ax=ax, handles=handles)

    impact_point = compute_impact_point(initial_position, initial_velocity)
    iter_time = time.time() - start
    cv2.waitKey(300)
    print(f"One RANSAC iter took {iter_time:.02} s, running @ {(1/iter_time):.02} Hz")
    if impact_point[1] > 0:
        print(f"IGNORE impact point below ground")
    else:
        print(f"Impact Point at Px={impact_point[0]} Py={impact_point[1]} Time: {t}.")

    cv2.imshow("Debug Image", debug_image)
    #ax[0].imshow(debug_image)

