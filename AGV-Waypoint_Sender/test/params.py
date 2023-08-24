from __future__ import annotations

from parameters.amcl import amcl
from parameters.controllers import *
from parameters.costmaps import *
from parameters.others import *
from parameters.planners import *


def _get_constant_params(planner_server):
    params = amcl
    params += bt_navigator
    params += bt_navigator_navigate_through_poses_rclcpp_node
    params += bt_navigator_navigate_to_pose_rclcpp_node
    params += local_costmap
    params += global_costmap
    params += map_server
    params += map_saver
    params += behavior_server
    params += robot_state_publisher
    params += waypoint_follower
    # params += velocity_smoother
    if planner_server == 'GridBased':
        params += navfn
    elif planner_server == 'SmacPlannerHybrid':
        params += smac_planner
    elif planner_server == 'ThetaStarPlanner':
        params += theta_star
    elif planner_server == 'SmacPlanner':
        params += smac_planner_2d
    return params


def get_params_VFO(planner_server, Kp, Lookahead, Rotate_to_heading):
    params = _get_constant_params(planner_server)
    vfo = VFO.format(
        Ka=float(2 * Kp),
        Kp=float(Kp),
        Lookahead=float(Lookahead),
        Rotate_to_heading=float(Rotate_to_heading),
    )
    params += CONTROLLER.format(FollowPath=vfo)
    return params


def get_params_DWB(planner_server):
    params = _get_constant_params(planner_server)
    dwb = DWB.format()
    params += CONTROLLER.format(FollowPath=dwb)
    return params


def get_params_NEO(planner_server, Lookahead=0.5, Rotate_to_heading=None):
    params = _get_constant_params(planner_server)
    neo = NEO.format(Lookahead=float(Lookahead))
    params += CONTROLLER.format(FollowPath=neo)
    return params


def get_params_MPPI(planner_server):
    params = _get_constant_params(planner_server)
    mppi = MPPI.format()
    params += CONTROLLER.format(FollowPath=mppi)
    return params


def get_params_RPP(planner_server, Lookahead=0.6, Rotate_to_heading=0.785):
    params = _get_constant_params(planner_server)
    rpp = RPP.format(
        Lookahead=float(Lookahead),
        Rotate_to_heading=float(Rotate_to_heading),
    )
    params += CONTROLLER.format(FollowPath=rpp)
    return params
