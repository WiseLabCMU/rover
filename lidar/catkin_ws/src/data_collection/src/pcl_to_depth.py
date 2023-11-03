#!/usr/bin/env python

import numpy as np
from scipy.ndimage.filters import gaussian_filter
import matplotlib.pyplot as plt

def normalization_255(distance):
    distance = abs(distance - 1)
    d_min = distance.min()
    d_max = distance.max()
    return (d_max - distance) / (d_max - d_min) * 1

h_res = [-60, 60]      # horizontal angle resolution in degrees
v_res = [-15, 15]      # verticle angle resolution in degrees

delta_h = 180.0 / 256
delta_v = 180.0 / 64

def pcl_to_depth_coordinate(points):
    x_points = points[:,0]
    y_points = points[:,1]
    z_points = points[:,2]

    #calculate point distance relative to origin in 3D
    distance = np.sqrt(x_points ** 2 + y_points ** 2 + z_points ** 2)

    alpha = np.degrees(np.arctan(y_points, x_points))
    beta = np.degrees(np.arcsin(z_points / distance))

    r = np.array(alpha / delta_h)
    c = np.array(beta / delta_v)

    points = np.stack((r, c))

    return (points, distance)

def normalize_to_zero(points):
    r_min = points[0].min()
    c_min = points[1].min()

    points[0] = points[0] + abs(r_min)
    points[1] = points[1] + abs(c_min)

    return points

def panoramic(coordinates, distance):
    image = np.zeros([64, 256], dtype = float)
    
    for i in range(0, len(distance)):
        c = int(coordinates[0][i])
        r = int(coordinates[1][i])
        d = distance[i]
        image[r][c] = d

    return image

def output_img(points):
    raw_points, distance = pcl_to_depth_coordinate(points)

    raw_points = np.floor(normalize_to_zero(raw_points))
    # remove points lies outside 64*256
    raw_points[0] = np.clip(raw_points[0], 0, 255)
    raw_points[1] = np.clip(raw_points[1], 0, 63)

    # normalize distance, points closer to radar is assigned higher value
    distance = normalization_255(distance)

    image = panoramic(raw_points, distance)
    blurred_image = gaussian_filter(image, sigma=2, mode='constant')

    # plt.imshow(np.squeeze(blurred_image), cmap='gray')
    # plt.colorbar()
    # plt.show()

    return blurred_image
