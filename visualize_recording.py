#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This file contains visualization function, to convert and display data from a .npz file, and is also
a script to replay the full content of a recording of a simulated sequence.
"""

import argparse

import cv2
import matplotlib.pyplot as plt
import numpy as np


def parse_args():
  """Arguments parsing function"""
  argparser = argparse.ArgumentParser()
  argparser.add_argument(
    "input_file",
    type=str,
    help="Path and name of the input .npz file (default: out.npz)")
  argparser.add_argument(
    "--dvs-resolution",
    default="1280x720",
    type=str,
    help="Resolution of the event camera (default: 1280x720)")
  return argparser.parse_args()


def viz_events(events, args):
  """Displays an image from the given events (positive events in red, negative ones in blue)"""
  image_resolution = args.dvs_resolution.split('x')
  dvs_img = np.zeros((int(image_resolution[1]), int(image_resolution[0]), 3), np.uint8)
  dvs_img[events[:]['y'], events[:]['x'], events[:]['pol'] * 2] = 255
  cv2.imshow("DVS", dvs_img)


def viz_depth_image(depth_image):
  """Converts, normalizes and displays the edge image"""
  color_map = plt.get_cmap("inferno")
  depth_image = depth_image.astype(np.float32)
  depth_image = ((depth_image[:, :, 2] + depth_image[:, :, 1]*256.0 + depth_image[:, :, 0]*256.0*256.0)/(256.0*256.0*256.0 - 1))
  depth_image_log = cv2.log(depth_image)
  depth_image_log_norm = np.zeros_like(depth_image_log)
  cv2.normalize(depth_image_log, depth_image_log_norm, 1.0, 0.0, cv2.NORM_MINMAX)
  depth_image_log_norm_inf = color_map(depth_image_log_norm)
  depth_image_log_norm_inf[:, :, :3] = depth_image_log_norm_inf[:, :, 2::-1] # RGBA -> BGRA
  cv2.imshow("Depth", depth_image_log_norm_inf)


def viz_point_cloud(point_cloud):
  """Creates a projection of the point cloud in an image and displays it"""

  # We get the color map we will use for the depths
  color_map = plt.get_cmap("inferno")

  # We create a false camera, of resolution 1280x720, with a 90° FOV, aligned with the LiDAR sensor
  # R_c_l is the rotation matrix from LiDAR to camera, to correct the axes
  f = 1280/(2*np.tan(90*np.pi/360))
  cx = 1280/2
  cy = 720/2
  K = np.array([[f, 0, cx],
                [0, f, cy],
                [0, 0, 1 ]])
  R_c_l = np.array([[0, 1, 0],
                    [0, 0, -1],
                    [1, 0, 0]])

  # We then filter the point cloud, to only retain points in front of the camera
  pcl_pts = point_cloud[:, :3]
  pcl_pts_filt = pcl_pts[pcl_pts[:, 0] > 0]

  # We project them to the camera's frame
  pcl_camera_frame = (R_c_l @ pcl_pts_filt.T).T
  depths = pcl_camera_frame[:, 2].copy()
  pcl_camera_frame[:, 0] /= depths
  pcl_camera_frame[:, 1] /= depths
  pcl_camera_frame[:, 2] /= depths

  # We project them in the image
  pcl_camera = (K @ pcl_camera_frame.T).T

  # We create the projection, and add the depth of each projected LiDAR point to it
  lidar_proj = np.zeros((720, 1280), dtype=np.float32)
  for i, pt in enumerate(pcl_camera[:, :2]):
    if pt[0] >= 0 and pt[0] < 1280 and pt[1] >= 0 and pt[1] < 720:
      cv2.circle(lidar_proj, (int(pt[0]), int(pt[1])), 1, depths[i], cv2.FILLED)
  lidar_proj /= np.max(lidar_proj)
  
  # We convert to the color map in use, and don't forget to convert from RGBA to BGRA
  lidar_img = color_map(lidar_proj)
  lidar_img[:, :, :3] = lidar_img[:, :, 2::-1]

  # We finish by displaying the image
  cv2.imshow("LiDAR", lidar_img)


def viz_rgb_image(rgb_image):
  """Displays the given RGB image"""
  cv2.imshow("RGB", rgb_image)


def main():
  """Main function"""

  # We begin by collecting the command-line args
  args = parse_args()

  # We load the sequence
  sequence = np.load(args.input_file, allow_pickle=True)

  # We copy all the elements needed from the sequence file into memory, for speed reasons
  events_with_ts = sequence["events"].copy()
  depth_images_with_ts = sequence["depth_images"].copy()
  lidar_clouds_with_ts = sequence["lidar_clouds"].copy()
  rgb_images_with_ts = sequence["rgb_images"].copy()

  # Once everything is read, we can close the file as it is no longer needed
  sequence.close()

  # We know that the events and depth images are produced at the highest frequency and are
  # synchronized: we display them at every time step, and check if a new LiDAR cloud or RGB image
  # was produced
  lidar_index = 0
  rgb_index = 0
  for i in range(events_with_ts.shape[0]):
    viz_events(events_with_ts[i, 0], args)
    viz_depth_image(depth_images_with_ts[i, 0])

    try:
      if lidar_clouds_with_ts[lidar_index, 1] <= events_with_ts[i, 1]:
        viz_point_cloud(lidar_clouds_with_ts[lidar_index, 0])
        lidar_index += 1
    except IndexError:
      pass

    try:
      if rgb_images_with_ts[rgb_index, 1] <= events_with_ts[i, 1]:
        viz_rgb_image(rgb_images_with_ts[rgb_index, 0])
        rgb_index += 1
    except IndexError:
      pass

    key = cv2.waitKey(0)
    if key == 27:
      break


if __name__ == "__main__":
  main()
