# Sensor Fusion (localization)
Two different algorithms are implemented for sensor fusion (extended Kalman filter and particle filter). Both algorithms were developed under the Udacity self-driving car engineer nano-degree.

## Overview
Kalman filters (including extended) and particle filters are algorithms used in sensor fusion. They are probabilistic models that estimate the location of a car in a given map. In this project two types of data are provided for the Kalman filter (LiDAR and RADAR), and object coordinates for the particle filter. Using these inputs, we can estimate where our vehicle (or any robot) is in a virtual world.

It should be noted that in both of these examples, we assume we have an understanding of the map that our vehicle is in. This is not always the case. In scenarios where the map is not known, we can use a SLAM algorithm that simultaneously localizes and maps the enviornment a vehicle is in. I have done a few projects involving SLAM, so feel free to check them out from my repository list.

## Dependencies Installation
Udacity provided me with some bash scripts to install all of the dependencies that you need to run both projects. If you are on a mac, then run the following command:
```bash
chmod +x install-scripts/install-mac.sh
./install-scripts/install-mac.sh
```
On linux:
```bash
chmod +x install-scripts/install-linux.sh
./install-scripts/install-linux.sh
```

## Build & Run
Both projects contains 3 scripts that you can use. The first script clean.sh deletes any old build environment. It is not compulsory but this can be useful is your system fails to build. The second script is the build.sh script. run this script to build your source code on your system. Then finally, the run.sh script can be used to test your code on a simulator provided by Udacity (click [here](https://github.com/udacity/self-driving-car-sim/releases) to install the simulator).
```bash
cd PROJECT
chmod +x clean.sh & ./clean.sh
chmod +x build.sh & ./build.sh
chmod +x run.sh & ./run.sh
```
Keep in mind that you only need to change access permissions (chmod) once. Some systems may not even require it. Also do not change the contents of CMakeList.txt (unless you know what you're doing).

## Kalman Filter


## Particle Filter
