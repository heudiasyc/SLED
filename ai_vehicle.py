#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
AIVehicle (AI-controlled vehicle) class definition.
"""

import random


class AIVehicle:
  """An AI-controlled vehicle, navigating freely in the simulated world"""

  # Class variable, that stores the reference to all the instances
  instances = []

  def __init__(self, transform, world, args):
    """
    Tries to spawn the vehicle at the given transform (may fail due to collision).
    If it succeeds, the vehicle is added to the instances list.
    """

    # We set the random seed
    random.seed = args.seed

    # We try to spawn the vehicle
    self.world = world
    self.vehicle = world.try_spawn_actor(self.get_random_blueprint(), transform)
    if self.vehicle is None:
      return
    
    # We enable its autopilot mode if required
    self.vehicle.set_autopilot(args.no_autopilot)

    # We register the instance
    AIVehicle.instances.append(self)


  def get_random_blueprint(self):
    """Gets a random vehicle blueprint"""
    blueprints = self.world.get_blueprint_library().filter("vehicle")
    return random.choice(blueprints)


  def destroy(self):
    """Destroys the vehicle"""
    self.vehicle.destroy()
