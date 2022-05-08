# Project: Particle Filter - Kidnapped Vehicle

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
The robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project we implement a 2 dimensional particle filter in C++. The particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step the filter also gets observation and control data.

The particle filter has to localize the robot as it moves across the map.

Getting Started
---

The project has been developed on a Linux machine with Python 3.6. The system was provided by Udacity for this particular project.

## Prerequisites
Following are the dependencies:

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's Term 2 Simulator. [Link](https://github.com/udacity/self-driving-car-sim/releases)

To install the dependencies, use the script [install-linux.sh](install-linux.sh)

Dataset
---
Synthetic data provided by Udacity is used for the project. The data is present in the [data](data) directory. It consists of measurements in a txt file format.

Using the application
---

## Build
Use the commands to build the project:

```bash
./clean.sh
./build.sh
```

## Run
After building the project, run the project:

```bash
./run.sh
```

## Results

- [Video](video.mp4) for the same

- [Youtube](https://youtu.be/Lqu65GlEvOs) video for the same

- The final errors are:

```
RMSE
x: 0.115
y: 0.110
yaw: 0.004
```