#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Arguments parser for the sequence generation script.
Allows to configure the simulation and all the sensors used.
"""

import argparse
from time import time


def parse_args():
  """Arguments parsing function"""
  argparser = argparse.ArgumentParser()
  argparser.add_argument(
    "--host",
    default="localhost",
    type=str,
    help="IP of the host server (default: localhost)")
  argparser.add_argument(
    "-p", "--port",
    default=2000,
    type=int,
    help="TCP port to listen to (default: 2000)")
  argparser.add_argument(
    "--traffic-manager-port",
    default=8000,
    type=int,
    help="TCP port of CARLA's traffic manager (default: 8000)")
  argparser.add_argument(
    "-m", "--map",
    default="Town01",
    type=str,
    help="Map name (default: Town01)")
  argparser.add_argument(
    "--hz",
    default=200.0,
    type=float,
    help="The fixed simulation frequency, used in particular by the event camera (default: 200.0)")
  argparser.add_argument(
    "--dvs-resolution",
    default="1280x720",
    type=str,
    help="Resolution of the event camera (default: 1280x720)")
  argparser.add_argument(
    "--dvs-fov",
    default="90",
    type=str,
    help="FOV of the event camera (default: 90°)")
  argparser.add_argument(
    "--lidar-hz",
    default=10.0,
    type=float,
    help="Number of full point clouds acquired by the LiDAR each second (default: 10.0)")
  argparser.add_argument(
    "--points-per-cloud",
    default=70000,
    type=int,
    help="LiDAR's points per cloud (default: 70000)")
  argparser.add_argument(
    "--range",
    default=200.0,
    type=float,
    help="LiDAR's maximum range in meters (default: 200.0)")
  argparser.add_argument(
    "--lidar-is-pandora",
    action="store_true",
    help="Should the LiDAR be based on the Pandora model?")
  argparser.add_argument(
    "--channels",
    default=40,
    type=int,
    help="LiDAR's channel count (default: 40)")
  argparser.add_argument(
    "--upper-fov",
    default=7.0,
    type=float,
    help="LiDAR's upper vertical fov angle in degrees (default: 7.0)")
  argparser.add_argument(
    "--lower-fov",
    default=-16.0,
    type=float,
    help="LiDAR's lower vertical fov angle in degrees (default: -16.0)")
  argparser.add_argument(
    "--rgb-fps",
    default=30.0,
    type=float,
    help="Number of frames acquired by the RGB camera per second (default: 30.0)")
  argparser.add_argument(
    "--rgb-resolution",
    default="1280x720",
    type=str,
    help="Resolution of the RGB camera (default: 1280x720)")
  argparser.add_argument(
    "--rgb-fov",
    default="90",
    type=str,
    help="FOV of the RGB camera (default: 90°)")
  argparser.add_argument(
    "--nvehicles",
    default=50,
    type=int,
    help="Number of other vehicles in the environment (default: 50)")
  argparser.add_argument(
    "--npedestrians",
    default=50,
    type=int,
    help="Number of pedestrians in the environment (default: 50)")
  argparser.add_argument(
    "--sun-altitude",
    default=90,
    type=int,
    help="Sun altitude (default: 90 (noon))")
  argparser.add_argument(
    "--cloudiness",
    default=0,
    type=int,
    help="Cloudiness (default: 0 (clear sky))")
  argparser.add_argument(
    "--no-autopilot",
    action="store_false",
    help="Disables the autopilot (the vehicles will remain stopped)")
  argparser.add_argument(
    "-o", "--output-file",
    default="data/out.npz",
    type=str,
    help="Path of the created output file (default: data/out.npz)")
  argparser.add_argument(
    "--csv-file",
    default="data/metadata.csv",
    type=str,
    help="Path of the created/updated csv metadata file (default: data/metadata.csv)")
  argparser.add_argument(
    "--duration",
    default=10.0,
    type=float,
    help="Duration of the sequence to record, in seconds (default: 10.0)")
  argparser.add_argument(
    "--discard-duration",
    default=3.0,
    type=float,
    help="Duration of the sequence discarded before starting the record (default: 3.0)")
  argparser.add_argument(
    "--seed",
    default=int(time()),
    type=int,
    help="Random seed for reproducibility (default: time.time())")
  argparser.add_argument(
    "--preview",
    action="store_true",
    help="Enables the live preview of the data being recorded (but slows down the whole process!)")
  argparser.add_argument(
    "--compressed",
    action="store_true",
    help="Enables data compression when saving on disk (slower but saves disk space, see README!)")
  return argparser.parse_args()
