# AGV-Simulation-ROS2-Gazebo


Build image and create container:

`./build_and_run.sh`

Start container:

`docker start agv_sim`

Attach to container:

`docker exec -it agv_sim bash`

Build ros2 package for simulator:

`. /root/workspace/ros2_build.sh`

Source installation:

`. /root/workspace/install/setup.bash`

Launch simulator:

`ros2 launch agv_simulation start_simulation.launch.py`

Rebuild model:

`ros2 run xacro xacro $PKG_DIR/urdf/agv.urdf.xacro > $PKG_DIR/urdf/agv.urdf`

`gz sdf -p  $PKG_DIR/urdf/agv.urdf > $PKG_DIR/models/agv.sdf`
