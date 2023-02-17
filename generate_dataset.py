#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This script can be used to generate the full SLED dataset in the CARLA simulator, composed of
N sequences, each containing:
- events from a neuromorphic camera;
- point clouds from a LiDAR;
- dense ground truth depths;
- images from a RGB camera.
Each sequence is saved in a single compressed .npz file.

This code was deeply inspired by the one of the Cooperative Driving Dataset (CODD):
https://github.com/eduardohenriquearnold/CODD
"""

import argparse
import csv
from datetime import datetime
from os.path import exists
import random
import subprocess


def parse_args():
  """Arguments parsing function"""
  argparser = argparse.ArgumentParser()
  argparser.add_argument(
    "--nb_seq",
    type=int,
    default=1,
    help="Number of sequences to randomly generate (default: 1). "
         "This argument is ignored if --seeds is set.")
  argparser.add_argument(
    "--seeds",
    type=str,
    default="",
    help="Path to a .csv file containing the seeds & maps to use (default: \"\"). "
         "If set, the number of sequences to generate is automatically determined from the content "
         "of the file")
  argparser.add_argument(
    "--outfolder",
    type=str,
    default="data",
    help="Path to the folder where generated sequences will be saved (default: data)")
  return argparser.parse_args()


def main():
  """Main function"""

  # We begin by collecting the command-line args
  args = parse_args()

  # If the user has provided a file containing a list of seeds, we must determine the number of
  # files to generate from it. If not, then we use args.nb_seq
  if args.seeds != "":
    with open(args.seeds, newline='') as f:
      csv_reader = csv.reader(f, delimiter=';')
      csv_lines = []
      for csv_line in csv_reader:
        # We only keep non-empty lines
        if csv_line:
          csv_lines.append(csv_line)
    nb_seq_to_generate = len(csv_lines)
  else:
    nb_seq_to_generate = args.nb_seq

  # We initialize the list of maps
  maps = [f"Town0{i}" for i in range(1, 8)] + ["Town10HD"]

  # For each of the N sequences...
  for i in range(nb_seq_to_generate):
    # Display progress to the user
    print(f"Generating sequence {i+1}/{nb_seq_to_generate}...")

    # If a seed file has been provided, then it is used for generating the sequence 
    # Otherwise, we select a seed based on current date and time, and select a map based on it
    # The seed/map is also saved as the output file name, for reproducibility purposes
    if args.seeds != "":
      out_name, seed, map, sun_alt, cloudiness = csv_lines[i]
      if map not in maps:
        raise Exception(f"Map {map} is not part of the list of maps!")
    else:
      seed = datetime.now().strftime("%Y%m%d_%H%M%S")
      random.seed(seed)
      map = random.choice(maps)
      out_name = f"{map}_{seed}"
      sun_alt = random.randint(-90, 90)
      cloudiness = random.randint(0, 100)

    # We set the adequate output file and csv file names
    output_file = f"{args.outfolder}/{out_name}.npz"
    csv_file = f"{args.outfolder}/metadata.csv"

    # We check if the file was not already generated; if it is the case we skip it
    if exists(output_file):
      print(f"File {output_file} already exists, skipping!")
      continue

    # And we finally call the sequence generation script with the args
    subprocess.run(["python3", "generate_sequence.py", "--map", map, "--seed", seed,
      "--lidar-is-pandora", "--sun-altitude", str(sun_alt), "--cloudiness", str(cloudiness),
      "--output-file", output_file, "--csv-file", csv_file, "--compressed"], check=True)


if __name__ == "__main__":
  main()
