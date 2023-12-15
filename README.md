# Roaming Stop (rstop) ROS Package

## Overview

The `roaming-stop` ROS package, abbreviated as `rstop`, is designed to facilitate emergency stopping of a mobile robot in roaming scenarios. The package includes an Android app, a web server-client, and a ROS node that communicates with a server to initiate a roaming stop.

## Features

- **Android App:** Provides a user-friendly interface to trigger a roaming stop remotely by making a socket connection.
- **Web Server-Client:** Enables communication between a website that can be accessed by a phone and the ROS server node.
- **ROS Node:** Listens for roaming stop requests, filters and adjusts the robot's velocity commands, and publishes safe zero velocity to stop the robot in emergency cases.

## Package Structure

The repository is organized as follows:

- `rstop`: Contains the source code for the Android app.
- `src/rstop/src/application`: Contains the code for the web server-client communication.
- `src/rstop`: ROS package directory with the server node code and related files.

##Usage
```bash
git clone https://github.com/AlphaGotReal/roaming-stop.git
cd roaming-stop
catkin build

#run the server as a website
rosrun rstop application

