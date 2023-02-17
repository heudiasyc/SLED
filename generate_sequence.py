#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This script can be used to generate a single sequence in the CARLA simulator, containing:
- events from a neuromorphic camera;
- point clouds from a LiDAR;
- dense ground truth depths;
- images from a RGB camera.
The sequence is saved in a single compressed .npz file.

This code was deeply inspired by the one of the Cooperative Driving Dataset (CODD):
https://github.com/eduardohenriquearnold/CODD
"""

import csv
import random
from time import sleep

import carla
import cv2
import numpy as np
from tqdm import tqdm

from ai_pedestrian import AIPedestrian
from ai_vehicle import AIVehicle
from ego_vehicle import EgoVehicle
from generate_sequence_args import parse_args
from visualize_recording import viz_depth_image, viz_events, viz_point_cloud, viz_rgb_image


def main():
  """Main function"""

  # We begin by collecting the command-line args
  args = parse_args()

  # We set the random seed
  random.seed(args.seed)

  # We connect to the CARLA simulator
  client = carla.Client(args.host, args.port)
  client.set_timeout(10.0)

  # We load the correct map
  world = client.load_world(args.map)

  # We apply the simulation settings
  settings = world.get_settings()
  settings.synchronous_mode = True
  settings.fixed_delta_seconds = 1.0/args.hz
  world.apply_settings(settings)

  # We configure the traffic manager
  traffic_manager = client.get_trafficmanager(args.traffic_manager_port)
  traffic_manager.set_synchronous_mode(True)
  traffic_manager.set_random_device_seed(args.seed)

  # We configure the time and weather settings
  weather = carla.WeatherParameters.ClearNoon
  weather.sun_altitude_angle = args.sun_altitude
  weather.cloudiness = args.cloudiness
  world.set_weather(weather)

  # We spawn the ego-vehicle
  ego_vehicle_transform = random.choice(world.get_map().get_spawn_points())
  ego_vehicle = EgoVehicle(ego_vehicle_transform, world, args)

  # We spawn the other AI-controlled vehicles
  while(len(AIVehicle.instances) < args.nvehicles):
    vehicle_transform = random.choice(world.get_map().get_spawn_points())
    AIVehicle(vehicle_transform, world, args)

  # We spawn the AI-controlled pedestrians
  while(len(AIPedestrian.instances) < args.npedestrians):
    pedestrian_location = world.get_random_location_from_navigation()
    pedestrian_transform = carla.Transform(location=pedestrian_location)
    AIPedestrian(pedestrian_transform, world, args)

  # We compute the number of world ticks we have to discard and to record, based on the durations
  # given by the user
  ticks_to_discard = int(args.discard_duration * args.hz)
  ticks_to_record = int(args.duration * args.hz)

  # Since we need to compute LiDAR point clouds at the full frequency but only keep a part of them
  # (more details on that topic further below), we have to compute the frequency at which we should
  # keep the LiDAR point clouds, as well as a variable to keep track of the current LiDAR point
  # cloud index
  keep_lidar_every_x_idx = args.hz // args.lidar_hz
  current_lidar_idx = 0

  # We create the queues, which will hold the numpy arrays of data before saving them to disk
  events_queue = []
  depth_images_queue = []
  lidar_clouds_queue = []
  rgb_images_queue = []

  try:
    # We compute a first world tick, after which we can enable the controller of the pedestrians
    world.tick()
    for pedestrian in AIPedestrian.instances:
      pedestrian.start_controller()

    # We loop a first time, to skip the first world ticks that should be discarded
    for _ in tqdm(range(ticks_to_discard), "Discarding ticks"):
      world.tick()

    # We clear the data that has been recorded by the sensors during that period
    ego_vehicle.clear_sensor_queues()

    # We then loop until we reach the end of the simulation
    for _ in tqdm(range(ticks_to_record), "Recording ticks"):
      world.tick()

      # If we get close to a red traffic light, we force it to go green
      if ego_vehicle.vehicle.is_at_traffic_light():
        traffic_light = ego_vehicle.vehicle.get_traffic_light()
        if traffic_light.get_state() == carla.TrafficLightState.Red:
          traffic_light.set_state(carla.TrafficLightState.Green)
          traffic_light.set_green_time(10.0)
          tqdm.write("Forced traffic light to go green (ego)")

      # If we are behind a vehicle which is close to a red traffic light, we also force it to go
      # green
      for vehicle in AIVehicle.instances:
        if vehicle.vehicle.is_at_traffic_light():
          ego_x = ego_vehicle.vehicle.get_transform().location.x
          ego_y = ego_vehicle.vehicle.get_transform().location.y
          ego_yaw = ego_vehicle.vehicle.get_transform().rotation.yaw
          other_x = vehicle.vehicle.get_transform().location.x
          other_y = vehicle.vehicle.get_transform().location.y
          other_yaw = vehicle.vehicle.get_transform().rotation.yaw
          dist = np.sqrt((ego_x-other_x)**2 + (ego_y-other_y)**2)
          yaw_diff = np.abs(ego_yaw-other_yaw)
          if dist < 20.0 and yaw_diff < 20.0:
            traffic_light = vehicle.vehicle.get_traffic_light()
            if traffic_light.get_state() == carla.TrafficLightState.Red:
              traffic_light.set_state(carla.TrafficLightState.Green)
              traffic_light.set_green_time(10.0)
              tqdm.write("Forced traffic light to go green (other)")

      # If events were generated, we convert, save, and optionally display them
      if ego_vehicle.dvs_queue:
        events = np.frombuffer(ego_vehicle.dvs_queue[0].raw_data, dtype=np.dtype([
          ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', bool)])).copy()
        events_queue.append(np.array([events, ego_vehicle.dvs_queue[0].timestamp], dtype=object))
        ego_vehicle.dvs_queue.pop(0)
        if args.preview:
          viz_events(events, args)

      # If a depth image was generated, we convert, save, and optionally display it
      if ego_vehicle.depth_queue:
        depth_image = np.frombuffer(ego_vehicle.depth_queue[0].raw_data, dtype=np.dtype("uint8")).copy()
        depth_image_res = args.dvs_resolution.split('x')
        depth_image = np.reshape(depth_image, (int(depth_image_res[1]), int(depth_image_res[0]), 4))
        depth_image = depth_image[:, :, :3]
        depth_images_queue.append(np.array([depth_image, ego_vehicle.depth_queue[0].timestamp], dtype=object))
        ego_vehicle.depth_queue.pop(0)
        if args.preview:
          viz_depth_image(depth_image)
      
      # If a full point cloud was generated, we convert, save, and optionally display it
      # Note: to ensure a good correlation between the depth images and the point clouds, we
      # generate the point clouds at the full frequency (otherwise CARLA would fragment them into
      # subpackets, and the movement of the vehicule would have to be compensated), but only keep
      # those corresponding to the desired LiDAR frequency
      # Other note: if the LiDAR sensor is based on the Pandora model, it is recreated by using 3
      # separate LiDARs in CARLA, which all store their point clouds in the queue (hence the "3*")
      if args.lidar_is_pandora:
        lidar_subpackets_per_pcl = 3
      else:
        lidar_subpackets_per_pcl = 1
      if len(ego_vehicle.lidar_queue) >= lidar_subpackets_per_pcl:
        pcl_pts_arr = []
        for i in range(lidar_subpackets_per_pcl):
          pcl_pts_tmp = np.frombuffer(ego_vehicle.lidar_queue[i].raw_data, dtype=np.dtype('f4')).copy()
          pcl_pts_tmp = np.reshape(pcl_pts_tmp, (-1, 4))
          pcl_pts_arr.append(pcl_pts_tmp)
        point_cloud = np.concatenate(pcl_pts_arr, axis=0)
        if current_lidar_idx % keep_lidar_every_x_idx == 0:
          lidar_clouds_queue.append(np.array([point_cloud, ego_vehicle.lidar_queue[0].timestamp], dtype=object))
        current_lidar_idx += 1
        for _ in range(lidar_subpackets_per_pcl):
          ego_vehicle.lidar_queue.pop(0)
        if args.preview:
          viz_point_cloud(point_cloud)

      # If a RGB image was generated, we convert, save, and optionally display it
      if ego_vehicle.rgb_queue:
        rgb_img = np.frombuffer(ego_vehicle.rgb_queue[0].raw_data, dtype=np.dtype("uint8")).copy()
        rgb_resolution = args.rgb_resolution.split('x')
        rgb_img = np.reshape(rgb_img, (int(rgb_resolution[1]), int(rgb_resolution[0]), 4))
        rgb_images_queue.append(np.array([rgb_img, ego_vehicle.rgb_queue[0].timestamp], dtype=object))
        ego_vehicle.rgb_queue.pop(0)
        if args.preview:
          viz_rgb_image(rgb_img)

      # If preview of the generated data is enabled, we must refresh the windows
      if args.preview:
        cv2.waitKey(1)

    # Once the end of the simulation is reached, we save the generated data and update the csv file
    tqdm.write(f"Finished recording, saving data to {args.output_file} (might take time!)...")

    if args.compressed:
      np.savez_compressed(args.output_file, events=events_queue, depth_images=depth_images_queue,
        lidar_clouds=lidar_clouds_queue, rgb_images=rgb_images_queue)
    else:
      np.savez(args.output_file, events=events_queue, depth_images=depth_images_queue,
        lidar_clouds=lidar_clouds_queue, rgb_images=rgb_images_queue)

    with open(args.csv_file, 'a', newline='') as csv_file:
      csv_writer = csv.writer(csv_file, delimiter=';')
      csv_writer.writerow([args.output_file.split('/')[-1], len(lidar_clouds_queue), args.map])

  # At the end (or if anything goes wrong), we remove all the vehicles/pedestrians from the
  # simulation
  finally:
    ego_vehicle.destroy()
    for vehicle in AIVehicle.instances:
      vehicle.destroy()
    for pedestrian in AIPedestrian.instances:
      pedestrian.destroy()

  # And for some reason, we have to wait for a few seconds to avoid having the process crashing with
  # a "terminate called without an active exception" error
  sleep(5)


if __name__ == "__main__":
  main()
