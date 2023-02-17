#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AIPedestrian (AI-controlled pedestrian) class definition.
"""

import random

import carla


class AIPedestrian:
  """An AI-controlled pedestrian, navigating freely in the simulated world"""

  # Class variable, that stores the reference to all the instances
  instances = []

  def __init__(self, transform, world, args):
    """
    Tries to spawn the pedestrian at the given transform (may fail due to collision).
    If it succeeds, the pedestrian is added to the instances list.
    """

    # We set the random seed
    random.seed = args.seed

    # We try to spawn the pedestrian
    self.world = world
    self.pedestrian = world.try_spawn_actor(self.get_random_blueprint(), transform)
    if self.pedestrian is None:
      return

    # We must add a controller to the pedestrian for it to walk in the world
    pedestrian_controller_bp = world.get_blueprint_library().find("controller.ai.walker")
    self.controller = world.spawn_actor(pedestrian_controller_bp, carla.Transform(), attach_to=self.pedestrian)

    # We register the instance
    AIPedestrian.instances.append(self)


  def get_random_blueprint(self):
    """Gets a random pedestrian blueprint"""
    blueprints = self.world.get_blueprint_library().filter("walker.pedestrian.*")
    return random.choice(blueprints)


  def start_controller(self):
    """Asks the pedestrian to start moving. This function must be called AFTER the first tick!"""
    self.controller.start()
    self.controller.go_to_location(self.world.get_random_location_from_navigation())


  def destroy(self):
    """Destroys the pedestrian (so long!)"""
    self.controller.stop()
    self.controller.destroy()
    self.pedestrian.destroy()
