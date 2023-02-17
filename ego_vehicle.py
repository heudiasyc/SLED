#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
EgoVehicle class definition.
"""

import random

import carla
import numpy as np


class EgoVehicle:
  """The ego-vehicle, which carries all the sensors for the recording"""

  # Class variable, which stores the reference to the unique instance (we want a single ego-vehicle)
  instance = None


  def __init__(self, transform, world, args):
    """
    Checks first if no other ego-vehicle has already been spawned.
    Then, tries to spawn the ego-vehicle at the given transform (may fail due to collision).
    If it succeeds, then adds all sensors to the vehicle, and sets the instance.
    """

    # We first check if no other ego-vehicle has already been spawned
    if EgoVehicle.instance is not None:
      raise Exception("An ego-vehicle already exists, no new ego-vehicle should be spawned!")

    # We set the random seed
    random.seed = args.seed

    # We try to spawn the ego-vehicle and to configure it
    self.world = world
    self.vehicle = world.try_spawn_actor(self.get_random_blueprint(), transform)
    if self.vehicle is None:
      raise Exception("Unable to spawn the ego-vehicle at the given transform!")
    self.vehicle.set_autopilot(args.no_autopilot)

    # We create the queues which will hold data from the sensors
    self.dvs_queue = []
    self.depth_queue = []
    self.lidar_queue = []
    self.rgb_queue = []

    # We save whether the LiDAR sensor is based on the Pandora model
    self.lidar_is_pandora = args.lidar_is_pandora

    # To avoid sensors seeing/hitting the roof of the vehicle, we must calculate a height margin
    # This margin is computed so that no LiDAR point of the lowest ring (lower fov angle of the
    # LiDAR, as set by the user) touches the roof of the vehicle
    height_margin = max(self.vehicle.bounding_box.extent.x, self.vehicle.bounding_box.extent.y) \
                    * np.tan(np.radians(-args.lower_fov))

    # We spawn the event camera
    dvs_transform = carla.Transform(carla.Location(z=2*self.vehicle.bounding_box.extent.z+height_margin))
    self.dvs = world.spawn_actor(self.get_dvs_bp(args), dvs_transform, attach_to=self.vehicle)
    self.dvs.listen(self.dvs_queue.append)

    # We spawn the depth sensor (at the exact same place as the event camera)
    depth_transform = carla.Transform(carla.Location(z=2*self.vehicle.bounding_box.extent.z+height_margin))
    self.depth = world.spawn_actor(self.get_depth_bp(args), depth_transform, attach_to=self.vehicle)
    self.depth.listen(self.depth_queue.append)

    # We spawn the LiDAR
    # Note: if the LiDAR sensor is based on the Pandora model, it is recreated by using 3 separate
    # LiDARs in CARLA (due to its repartition of channels: more dense in the center, more sparse for
    # the top and bottom channels). All 3 of them save their results in the `lidar_queue` array
    lidar_transform = carla.Transform(carla.Location(z=2*self.vehicle.bounding_box.extent.z+height_margin))
    if self.lidar_is_pandora:
      points_per_cloud_top = args.points_per_cloud * 5/40
      points_per_cloud_mid = args.points_per_cloud * 25/40
      points_per_cloud_bot = args.points_per_cloud * 10/40
      self.lidar = []
      self.lidar.append(world.spawn_actor(self.get_lidar_bp(args.hz, points_per_cloud_top,
        args.range, 5, 7.0, 3.0), lidar_transform, attach_to=self.vehicle))
      self.lidar.append(world.spawn_actor(self.get_lidar_bp(args.hz, points_per_cloud_mid,
        args.range, 25, 2.0, -6.0), lidar_transform, attach_to=self.vehicle))
      self.lidar.append(world.spawn_actor(self.get_lidar_bp(args.hz, points_per_cloud_bot,
        args.range, 10, -7.0, -16.0), lidar_transform, attach_to=self.vehicle))
      for lidar_sensor in self.lidar:
        lidar_sensor.listen(self.lidar_queue.append)
    else:
      self.lidar = world.spawn_actor(self.get_lidar_bp(args.lidar_hz, args.points_per_cloud,
        args.range, args.channels, args.upper_fov, args.lower_fov), lidar_transform, attach_to=self.vehicle)
      self.lidar.listen(self.lidar_queue.append)

    # We spawn the RGB camera
    rgb_transform = carla.Transform(carla.Location(z=2*self.vehicle.bounding_box.extent.z+height_margin))
    self.rgb = world.spawn_actor(self.get_rgb_bp(args), rgb_transform, attach_to=self.vehicle)
    self.rgb.listen(self.rgb_queue.append)

    # We register the instance
    EgoVehicle.instance = self


  def get_random_blueprint(self):
    """Gets a random car blueprint"""
    blueprints = self.world.get_blueprint_library().filter("vehicle")
    blueprints = [x for x in blueprints if int(x.get_attribute("number_of_wheels")) == 4]
    blueprints = [x for x in blueprints if not x.id.endswith(("carlacola", "firetruck", "ambulance",
                                                              "sprinter", "t2", "t2_2021",
                                                              "fusorosa"))]
    return random.choice(blueprints)


  def get_dvs_bp(self, args):
    """Gets and configures the DVS camera blueprint"""
    bp = self.world.get_blueprint_library().find("sensor.camera.dvs")
    dvs_resolution = args.dvs_resolution.split('x')
    bp.set_attribute("image_size_x", dvs_resolution[0])
    bp.set_attribute("image_size_y", dvs_resolution[1])
    bp.set_attribute("fov", args.dvs_fov)
    bp.set_attribute("gamma", "2.2") # See https://github.com/carla-simulator/carla/issues/6103
    bp.set_attribute("motion_blur_intensity", "0.0")
    bp.set_attribute("use_log", "False")
    bp.set_attribute("positive_threshold", "10")
    bp.set_attribute("negative_threshold", "10")
    bp.set_attribute("sensor_tick", "0.0")
    return bp


  def get_depth_bp(self, args):
    """Gets and configures the depth camera blueprint"""
    bp = self.world.get_blueprint_library().find("sensor.camera.depth")
    depth_resolution = args.dvs_resolution.split('x')
    bp.set_attribute("image_size_x", depth_resolution[0])
    bp.set_attribute("image_size_y", depth_resolution[1])
    bp.set_attribute("fov", args.dvs_fov)
    bp.set_attribute("sensor_tick", "0.0")
    return bp


  def get_lidar_bp(self, lidar_hz, points_per_cloud, range, channels, upper_fov, lower_fov):
    """Gets and configures the LiDAR blueprint"""
    bp = self.world.get_blueprint_library().find("sensor.lidar.ray_cast")
    bp.set_attribute("rotation_frequency", str(lidar_hz))
    bp.set_attribute("points_per_second", str(lidar_hz*points_per_cloud))
    bp.set_attribute("range", str(range))
    bp.set_attribute("channels", str(channels))
    bp.set_attribute("upper_fov", str(upper_fov))
    bp.set_attribute("lower_fov", str(lower_fov))
    bp.set_attribute("dropoff_general_rate", "0.0")
    bp.set_attribute("dropoff_intensity_limit", "1.0")
    bp.set_attribute("dropoff_zero_intensity", "0.0")
    bp.set_attribute("sensor_tick", "0.0")
    return bp


  def get_rgb_bp(self, args):
    """Gets and configures the RGB camera blueprint"""
    bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
    rgb_resolution = args.rgb_resolution.split('x')
    bp.set_attribute("image_size_x", rgb_resolution[0])
    bp.set_attribute("image_size_y", rgb_resolution[1])
    bp.set_attribute("fov", args.rgb_fov)
    bp.set_attribute("enable_postprocess_effects", "True")
    bp.set_attribute("gamma", "2.2") # See https://github.com/carla-simulator/carla/issues/6103
    bp.set_attribute("sensor_tick", str(1/args.rgb_fps))
    return bp


  def clear_sensor_queues(self):
    """Clears the data queue for each sensor"""
    self.dvs_queue.clear()
    self.depth_queue.clear()
    self.lidar_queue.clear()
    self.rgb_queue.clear()


  def destroy(self):
    """Destroys the sensors and the ego-vehicle"""
    self.dvs.destroy()
    self.depth.destroy()
    if self.lidar_is_pandora:
      for lidar_sensor in self.lidar:
        lidar_sensor.destroy()
    else:
      self.lidar.destroy()
    self.rgb.destroy()
    self.vehicle.destroy()
