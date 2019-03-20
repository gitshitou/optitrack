ROS OptiTrack Client for Motive 2
=================================

## Running Client Node

```bash
rosrun optitrack_motive_2_client optitrack_motive_2_client_node --server 192.168.1.12 --local 192.168.1.123
```

## Debugging Client Node

```bash
# From root of workspace
catkin_make -DCMAKE_BUILD_TYPE=Debug

gdb --args `catkin_find optitrack_motive_2_client optitrack_motive_2_client_node` --server 192.168.1.12 --local 192.168.1.123
# Common gdb commands
> run
> bt
> quit
```

## Using the ROS Launch File

```bash
# Once package has been added to workspace
roslaunch optitrack_motive_2_client optitrack.launch local:=[local IP address] server:=192.168.1.12
```
