# ros_assignment

Robot collaboration for fruit detection and counting in the **LCAS** Bacchus vineyard simulation.

## Instructions
Create a fresh catkin workspace and clone the necessary repos:
```bash
mkdir -p ~/ros_ws/src && cd ros_ws/src

git clone https://github.com/nikostsagk/ros_assignment.git
git clone -b ros_assignment https://github.com/nikostsagk/bacchus_lcas.git
git clone -b ros_assignment https://github.com/nikostsagk/cmp9767m.git

cd ..
catkin_make -j12
source ~/ros_ws/devel/setup.bash
```

## How to run

### Running a single Thorvald
```bash
# Launch the gazebo simulation
roslaunch ros_assignment thorvald_bringup.launch fake_localisation:=false multi_sim:=false

# Launch the detection server and the detection client
# (on a new terminal)
roslaunch ros_assignment ros_assignment.launch nr:=1 nav_client_file:=$(rospack find ros_assignment)/config/navigation_goals.yaml
```

### Running 7 Thorvalds!
To run 7 Thorvalds ensure that your system is powerful enough and if it crashes try launching again 🙃
```bash
# Launch the gazebo simulation
roslaunch ros_assignment thorvald_bringup.launch fake_localisation:=false multi_sim:=true

# Launch the detection server and the detection client
# (on a new terminal)
roslaunch ros_assignment ros_assignment.launch nr:=7 nav_client_file:=$(rospack find ros_assignment)/config/multi_navigation_goals.yaml
```

## Description
This package provides a detection service (`getGrapes`), which given an RGB+depth pair, returns grape detections in real world coordinates (using the camera's `frame_id`).
At the same time, the service aggregates all the detections over time and publishes them in a latched `markerArray` topic using `/map` as `frame_id`.

It also provides an actionServer client, that uses a `topological_navigation` map to move the robot across the field. On each node, the robot stops and calls the
`getGrapes` service.

The nodes can be modified under `config/navigation_goals.yaml`.

## Known issues:
When its used with multiple Thorvalds (e.g. 7 robots on the field) sometimes the Gazebo server crashes. Also tf extrapolation error might occur,
as the system struggles managing the load of the cameras. It's strongly recommended to be run on a powerful machine.

## Troubleshooting
```bash
sudo reboot # and pray
```
