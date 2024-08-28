# GPS (zed-f9p)
# PyQt5 for Navigation with Openrouteservice API
# azimuth(bearing) calculation
1. get gps data from zed-f9p
2. calculate current azimuth when it's moving
3. calculate target azimuth with waypoints
# Pure Pursuit algorithm for Direction
# ROS2 communication

# How to run
1. without Openrouteservice API
First, turn on ROS2
- source your_ros_ws/install/setup.bash
- cd src/GPS_navigation/GPS_navigation (where python files are.)

Run the files with this order
[openrouteservice.py -> zed_f9p.py -> purepursuit.py]

2. with Openrouteservice API

Run the file [openrouteservice_navigation_api.py] in the folder openrouteservice-py
