# GPS (zed-f9p)
# PyQt5 for Navigation with Openrouteservice API
# Pure Pursuit algorithm for Direction
# ROS2 communication

# How to run
1. without Openrouteservice API
First, turn on ROS2
<cd your_ros_ws>
<source install/setup.bash>
<cd src/GPS_navigation/GPS_navigation> : where python files are.

Run the files with this order
[openrouteservice.py -> zed_f9p.py -> purepursuit.py]

2. with Openrouteservice API
Run the file [openrouteservice_navigation_api.py] in the folder openrouteservice-py
