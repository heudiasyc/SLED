# The Synthetic LiDAR Events Depths (SLED) Dataset

![Dataset teaser (day)](gifs/teaser_sled_day.gif)\
![Dataset teaser (night)](gifs/teaser_sled_night.gif)

This repository holds the code to generate the Synthetic LiDAR Events Depths (SLED) dataset, as described in the "Learning to Estimate Two Dense Depths from LiDAR and Event Data" article. If you use this code as part of your work, please cite:

```BibTeX
@inproceedings{Brebion2023LearningTE,
  title={Learning to Estimate Two Dense Depths from {LiDAR} and Event Data},
  author={Vincent Brebion and Julien Moreau and Franck Davoine},
  booktitle={Image Analysis - 22nd Scandinavian Conference, {SCIA} 2023, Sirkka, Finland, April 18-21, 2023, Proceedings, Part {II}},
  series={Lecture Notes in Computer Science},
  volume={13886},
  publisher={Springer},
  pages={517-533},
  year={2023}
}
```

Parts of this code were originally inspired by the one of the Cooperative Driving Dataset (<https://github.com/eduardohenriquearnold/CODD>).

## Downloading the dataset

If you do not wish to regenerate the dataset, you can download it directly from the [following webpage](https://datasets.hds.utc.fr/share/g1z78rPKp1ykVe6).

## Installation

### Dependencies

**Note:** we recommend using `micromamba` (<https://github.com/mamba-org/mamba>) as a lighter and much faster alternative to `conda`/`miniconda`.\
However, you can safely replace `micromamba` by `conda` in the following commands if you prefer!

First of all, install CARLA (**version 0.9.14**) and its additional assets by following the official documentation (<https://carla.readthedocs.io/en/0.9.14/start_quickstart/#carla-installation> and <https://carla.readthedocs.io/en/0.9.14/start_quickstart/#import-additional-assets>).

Then, to install the CARLA client library and our dependencies, create a micromamba environment as follows:

```txt
micromamba create --name sled
micromamba activate sled
micromamba install python=3.8 matplotlib opencv tqdm -c conda-forge
pip3 install -U carla==0.9.14
```

### Tweaking NumPy's data compression

Generating the full dataset without compression will result in a size on disk between 1.5 and 1.6 TB. In order to save some space, we save generated data as a compressed version, resulting in a size on disk between 500 and 600 GB.

However, when trying to save data with compression enabled in NumPy, a default high compression level is used, which is very time-consuming.

Therefore, following [this issue](https://github.com/numpy/numpy/issues/20995), we encourage you to change the compression level to a lower value.\
To do so, as this feature is not currently natively supported in NumPy, you will need to manually modify the `numpy/lib/npyio.py` file (in our case, this file was located in `~/micromamba/pkgs/numpy-1.24.1-py38hab0fcb9_0/lib/python3.8/site-packages/numpy/lib/npyio.py`).

Once found, replace the following line:

```py
zipf = zipfile_factory(file, mode="w", compression=compression)
```

with:

```py
zipf = zipfile_factory(file, mode="w", compression=compression, compresslevel=1)
```

We use here a compression level of 1, as higher compression levels are much more time consuming and do not reduce dataset size by much.

## Generating the dataset

In order to generate the dataset, you first need to launch the CARLA simulator. In a first terminal, navigate to the install location of CARLA, and use the following command:

```txt
./CarlaUE4.sh -RenderOffScreen -nosound
```

Once done, you can then launch the full dataset generation (in a second terminal):

```txt
micromamba activate sled
python3 generate_dataset.py --seeds data/seeds_sled.csv --outfolder data/
```

**Note:** this will generate the full dataset on all maps. If you need to split it into train/val/test sets, you must do it manually by moving the files in separate folders after they are generated, and by duplicating the `metadata.csv` file in each folder and only keeping the correct entries in each of them.

**Note:** CARLA tends to crash quite often, so check regularly the progress of the dataset generation!\
After a crash, simply relaunch the simulator and the dataset generation script (all already recorded sequences will be automatically skipped).

## Visualizing the data

If you wish to visualize a sequence (either one you generated or one you downloaded), you can use the following command:

```txt
micromamba activate sled
python3 visualize_recording.py path/to/recording.npz
```

It should open four windows, displaying the LiDAR point clouds, events, ground truth depths, and RGB images.

## Dataset format

Each sequence is recorded as a compressed .npz NumPy file, which can be loaded as follows:

```py
import numpy as np
sequence = np.load("path/to/file.npz", allow_pickle=True)
```

Each sequence contains 4 distinct data arrays, for the events, LiDAR point clouds, RGB images, and depth images.

### Events

Event data can be accessed as follows:

```py
events_with_ts = sequence["events"].copy()
```

`events_with_ts` is a NumPy array of shape `(N_ep, 2)`, where `N_ep` is the number of event packets in the array, and where `2` indicates two dimensions:

- the first dimension contains the event-based data, where each of the `N_ep` packets contains `N_e` events, each with the following data:
  - `x`: x pixel coordinate, saved as a `np.uint16`;
  - `y`: y pixel coordinate, saved as a `np.uint16`;
  - `t`: timestamp of the event, saved as a `np.int64`;
  - `pol`: polarity of the event, saved as a `bool`;
- the second dimension contains the timestamps (in seconds) at which the data was produced in CARLA (for synchronization with the other sensors), expressed as floats.

For instance, if you want to read the polarity of the very first event in the sequence, you can use the following code:

```py
# Detailed
first_event_packet = events_with_ts[0, 0]
first_event_in_the_packet = first_event_packet[0]
polarity = first_event_in_the_packet["pol"]

# Or in short
polarity = events_with_ts[0, 0][0]["pol"]
```

And if you want to read the CARLA timestamp of the first event packet:

```py
carla_ts = events_with_ts[0, 1]
```

### LiDAR point clouds

LiDAR data can be accessed as follows:

```py
lidar_clouds_with_ts = sequence["lidar_clouds"].copy()
```

`lidar_clouds_with_ts` is a NumPy array of shape `(N_lp, 2)`, where `N_lp` is the number of LiDAR point clouds in the array, and where `2` indicates two dimensions:

- the first dimension contains the LiDAR-based data, where each of the `N_lp` point clouds contains `N_l` LiDAR points, each with 4 `float32` elements:
  - the first one is the `x` position;
  - the second one is the `y` position;
  - the third one is the `z` position;
  - the fourth and final one is the intensity of the point (in the `[0.0, 1.0]` range);
- the second dimension contains the timestamps (in seconds) at which the data was produced in CARLA (for synchronization with the other sensors), expressed as floats.

For instance, if you want to read the `z` position of the very first LiDAR point in the sequence, you can use the following code:

```py
# Detailed
first_lidar_cloud = lidar_clouds_with_ts[0, 0]
first_point_in_the_cloud = first_lidar_cloud[0]
z_pos = first_point_in_the_cloud[2]

# Or in short
z_pos = lidar_clouds_with_ts[0, 0][0][2]
```

### RGB images

RGB data can be accessed as follows:

```py
rgb_images_with_ts = sequence["rgb_images"].copy()
```

`rgb_images_with_ts` is a NumPy array of shape `(N_i, 2)`, where `N_i` is the number of RGB images in the array, and where `2` indicates two dimensions:

- the first dimension contains the RGB data, where each of the `N_i` images is a NumPy array of type `np.uint8` and of shape `(H, W, 4)` (`4` because images are saved with the BGRA format);
- the second dimension contains the timestamps (in seconds) at which the data was produced in CARLA (for synchronization with the other sensors), expressed as floats.

For instance, if you want to read the green value of the top left pixel of the very first image in the sequence, you can use the following code:

```py
# Detailed
first_image = rgb_images_with_ts[0, 0]
top_left_pixel = first_image[0, 0]
green_component = top_left_pixel[1]

# Or in short
green_component = rgb_images_with_ts[0, 0][0, 0, 1]
```

### Depth images

Depth data can be accessed as follows:

```py
depth_images_with_ts = sequence["depth_images"].copy()
```

`depth_images_with_ts` is a NumPy array of shape `(N_d, 2)`, where `N_d` is the number of depth images in the array, and where `2` indicates two dimensions:

- the first dimension contains the depth data, where each of the `N_d` depth images is a NumPy array of type `np.uint8` and of shape `(H, W, 4)` (`4` because images are saved with the BGRA format)
  - be careful, depth data is encoded over the RGB channels of the image, and can be recovered as follows:

    ```py
    depth_in_meters = 1000 * ((R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1))
    ```

- the second dimension contains the timestamps (in seconds) at which the data was produced in CARLA (for synchronization with the other sensors), expressed as floats.

For instance, if you want to read the depth value of the top left pixel of the very first depth image in the sequence, you can use the following code:

```py
# Get the depth image
depth_image = depth_images_with_ts[0, 0]

# Convert the image to a single channel image with float32 values (depths in meters)
depth_image = depth_image.astype(np.float32)
depth_image = ((depth_image[:, :, 2] + depth_image[:, :, 1]*256.0 + depth_image[:, :, 0]*256.0*256.0)/(256.0*256.0*256.0 - 1))
depth_image *= 1000

# Get the value for the top left pixel
depth_val = depth_image[0, 0]
```

## Code details

The code was developed so as to be as simple and as clean as possible. Each file and each function was properly documented, so do not hesitate to take a look at them!
