#!/bin/bash

ros2 run xacro xacro $PKG_DIR/urdf/agv.urdf.xacro > $PKG_DIR/urdf/agv.urdf

gz sdf -p $PKG_DIR/urdf/agv.urdf > $PKG_DIR/models/agv/agv.sdf