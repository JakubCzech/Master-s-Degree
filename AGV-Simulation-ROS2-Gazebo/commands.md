For run on your hardware:

Go to urdf/agv.urdf.xacro Line 30 and change filepath

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

`. /root/workspace/rebuild_model.sh`

Find not responsing gazebo

`ps aux | grep gzserver`

Launch rqt for visualization

`rqt_graph --force-discover`
