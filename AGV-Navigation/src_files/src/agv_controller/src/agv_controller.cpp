#include "agv_controller/agv_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d; // NOLINT
using rcl_interfaces::msg::ParameterType;

namespace agv_controller
{

    void AGVController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        auto node = parent.lock();
        node_ = parent;
        if (!node)
        {
            throw nav2_core::PlannerException("Unable to lock node!");
        }

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        tf_ = tf;
        plugin_name_ = name;
        logger_ = node->get_logger();
        clock_ = node->get_clock();

        double transform_tolerance = 0.1;
        double control_frequency = 20.0;
        goal_dist_tol_ = 0.25; // reasonable default before first update

        declare_parameter_if_not_declared(
            node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.6));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(0.9));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".lookahead_time", rclcpp::ParameterValue(1.5));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".rotate_to_heading_angular_vel", rclcpp::ParameterValue(1.8));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".use_velocity_scaled_lookahead_dist",
            rclcpp::ParameterValue(false));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".min_approach_linear_velocity", rclcpp::ParameterValue(0.05));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".approach_velocity_scaling_dist",
            rclcpp::ParameterValue(0.6));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
            rclcpp::ParameterValue(1.0));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".use_collision_detection",
            rclcpp::ParameterValue(true));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".use_regulated_linear_velocity_scaling", rclcpp::ParameterValue(true));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
            rclcpp::ParameterValue(true));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".cost_scaling_dist", rclcpp::ParameterValue(0.6));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".cost_scaling_gain", rclcpp::ParameterValue(1.0));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".inflation_cost_scaling_factor", rclcpp::ParameterValue(3.0));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".regulated_linear_scaling_min_radius", rclcpp::ParameterValue(0.90));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".regulated_linear_scaling_min_speed", rclcpp::ParameterValue(0.25));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".use_rotate_to_heading", rclcpp::ParameterValue(true));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".rotate_to_heading_min_angle", rclcpp::ParameterValue(0.785));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_angular_accel", rclcpp::ParameterValue(3.2));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".allow_reversing", rclcpp::ParameterValue(false));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_robot_pose_search_dist",
            rclcpp::ParameterValue(getCostmapMaxExtent()));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".use_interpolation",
            rclcpp::ParameterValue(true));


        // VFO parameters
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".Kp", rclcpp::ParameterValue(1.0));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".Ka", rclcpp::ParameterValue(2.0));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_speed", rclcpp::ParameterValue(0.25));
        node->get_parameter(plugin_name_ + ".Kp", Kp);
        node->get_parameter(plugin_name_ + ".Ka", Ka);
        node->get_parameter(plugin_name_ + ".max_speed", max_speed);
        // log VFO parameters
        last_Phi = 0.0;
        last_vx = 0.0;
        last_vy = 0.0;
        last_x_d = 0.0;
        last_y_d = 0.0;
        // End of VFO parameters

        node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
        base_desired_linear_vel_ = desired_linear_vel_;
        node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);

        node->get_parameter(plugin_name_ + ".min_lookahead_dist", min_lookahead_dist_);
        node->get_parameter(plugin_name_ + ".max_lookahead_dist", max_lookahead_dist_);
        node->get_parameter(plugin_name_ + ".lookahead_time", lookahead_time_);
        node->get_parameter(
            plugin_name_ + ".rotate_to_heading_angular_vel",
            rotate_to_heading_angular_vel_);
        node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
        node->get_parameter(
            plugin_name_ + ".use_velocity_scaled_lookahead_dist",
            use_velocity_scaled_lookahead_dist_);
        node->get_parameter(
            plugin_name_ + ".min_approach_linear_velocity",
            min_approach_linear_velocity_);
        node->get_parameter(
            plugin_name_ + ".approach_velocity_scaling_dist",
            approach_velocity_scaling_dist_);
        if (approach_velocity_scaling_dist_ > costmap_->getSizeInMetersX() / 2.0)
        {
            RCLCPP_WARN(
                logger_, "approach_velocity_scaling_dist is larger than forward costmap extent, "
                         "leading to permanent slowdown");
        }
        node->get_parameter(
            plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot",
            max_allowed_time_to_collision_up_to_carrot_);
        node->get_parameter(
            plugin_name_ + ".use_collision_detection",
            use_collision_detection_);
        node->get_parameter(
            plugin_name_ + ".use_regulated_linear_velocity_scaling",
            use_regulated_linear_velocity_scaling_);
        node->get_parameter(
            plugin_name_ + ".use_cost_regulated_linear_velocity_scaling",
            use_cost_regulated_linear_velocity_scaling_);
        node->get_parameter(plugin_name_ + ".cost_scaling_dist", cost_scaling_dist_);
        node->get_parameter(plugin_name_ + ".cost_scaling_gain", cost_scaling_gain_);
        node->get_parameter(
            plugin_name_ + ".inflation_cost_scaling_factor",
            inflation_cost_scaling_factor_);
        node->get_parameter(
            plugin_name_ + ".regulated_linear_scaling_min_radius",
            regulated_linear_scaling_min_radius_);
        node->get_parameter(
            plugin_name_ + ".regulated_linear_scaling_min_speed",
            regulated_linear_scaling_min_speed_);
        node->get_parameter(plugin_name_ + ".use_rotate_to_heading", use_rotate_to_heading_);
        node->get_parameter(plugin_name_ + ".rotate_to_heading_min_angle", rotate_to_heading_min_angle_);
        node->get_parameter(plugin_name_ + ".max_angular_accel", max_angular_accel_);
        node->get_parameter(plugin_name_ + ".allow_reversing", allow_reversing_);
        node->get_parameter("controller_frequency", control_frequency);
        node->get_parameter(
            plugin_name_ + ".max_robot_pose_search_dist",
            max_robot_pose_search_dist_);
        node->get_parameter(
            plugin_name_ + ".use_interpolation",
            use_interpolation_);

        transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
        control_duration_ = 1.0 / control_frequency;

        if (inflation_cost_scaling_factor_ <= 0.0)
        {
            RCLCPP_WARN(
                logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
                         "it should be >0. Disabling cost regulated linear velocity scaling.");
            use_cost_regulated_linear_velocity_scaling_ = false;
        }

        /** Possible to drive in reverse direction if and only if
         "use_rotate_to_heading" parameter is set to false **/

        if (use_rotate_to_heading_ && allow_reversing_)
        {
            RCLCPP_WARN(
                logger_, "Disabling reversing. Both use_rotate_to_heading and allow_reversing "
                         "parameter cannot be set to true. By default setting use_rotate_to_heading true");
            allow_reversing_ = false;
        }

        global_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
        carrot_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("lookahead_point", 1);
        carrot_arc_pub_ = node->create_publisher<nav_msgs::msg::Path>("lookahead_collision_arc", 1);
        vfo_param_pub_ = node->create_publisher<std_msgs::msg::Float32MultiArray>("vfo", 1);
        // initialize collision checker and set costmap
        collision_checker_ = std::make_unique<nav2_costmap_2d::
                                                  FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
        collision_checker_->setCostmap(costmap_);

        RCLCPP_INFO(logger_, "+++++++++++++++++++++++++++++++++Kp: %f, Ka: %f, lookahead_dist: %f, rotate_to_heading_min_angle %f", Kp, Ka, lookahead_dist_, rotate_to_heading_min_angle_);

    }

    // VFO functions
        std::array<double, 2> AGVController::VFO_Point_follow(const double e_x,const double e_y,const double theta_d)
    {
        double theta=0.0;
        if (e_x0 == 0.0 && e_y0 == 0.0)
        {

            e_x0 = e_x;
            e_y0 = e_y;
            e_x0_d = e_x0 * cos(theta_d) + e_y0 * sin(theta_d);
        }

        // Common VFO
        double e_mod = sqrt(pow(e_x, 2) + pow(e_y, 2));
        // End Common VFO

        double n = 0.2;
        double v_x = -1 * n * sgn(e_x0_d) * e_mod * cos(theta_d);
        double v_y = -1 * n * sgn(e_x0_d) * e_mod * sin(theta_d);

        // Common VFO
        double h_x = (Kp * e_x) + v_x;
        double h_y = (Kp * e_y) + v_y;

        double theta_a,theta_a_dot;
        double eq, h_x_dot, h_y_dot;

        if (e_mod < 0.1)
        {
            theta_a = theta_d;
            theta_a_dot = 0;
            u2 = 0;
        }
        else
        {
            u2 = h_x * cos(theta) + h_y * sin(theta);
            eq = e_x * u2 * cos(theta) + e_y * u2 * sin(theta);
            h_x_dot = (-1 * Kp * u2 * cos(theta)) + (n * sgn(e_x0_d) * eq / e_mod * cos(theta_d));
            h_y_dot = (-1 * Kp * u2 * sin(theta)) + (n * sgn(e_x0_d) * eq / e_mod * sin(theta_d));
            // Common VFO
            theta_a = atan2c(sgn(e_x0_d) * h_y, sgn(e_x0_d) * h_x);
            theta_a_dot = (h_y_dot * h_x - h_y * h_x_dot) / (pow(h_x, 2) + pow(h_y, 2));
            // End Common VFO
        }
        last_Phi = theta_a;

        // Common VFO
        u1 = Ka * (theta_a - theta) + theta_a_dot;
        // End Common VFO
        std_msgs::msg::Float32MultiArray msg;
        msg.data.push_back(e_x);
        msg.data.push_back(e_y);
        msg.data.push_back(e_x0_d);
        msg.data.push_back(e_mod);
        msg.data.push_back(h_x);
        msg.data.push_back(h_y);
        msg.data.push_back(v_x);
        msg.data.push_back(v_y);
        msg.data.push_back(theta_a);
        msg.data.push_back(theta_a_dot);
        msg.data.push_back(u1);
        msg.data.push_back(u2);


        vfo_param_pub_->publish(msg);

        RCLCPP_DEBUG(logger_, "E: %.3f,%.3f | ex_0d: %.3f |  e_mod: %.3f, |  h: %.3f,%.3f, |  v: %.3f,%.3f, |  theta_a: %.3f, | theta_a_dot: %.3f, | u: %.3f,%.3f",e_x, e_y, e_x0_d, e_mod, h_x, h_y, v_x, v_y, theta_a, theta_a_dot, u1, u2);
        return std::array<double, 2>{u1, u2};
    }

        std::array<double, 2> AGVController::VFO_Traj_follow(const double e_x, const double e_y, const double theta_d)
    {
        double theta =0.0;
        // Common VFO
        double e_mod = sqrt(pow(e_x, 2) + pow(e_y, 2));
        // End Common VFO

        double sigma = 1.0;
        double v_x = (e_x - last_x_d) / control_duration_;
        double v_y = (e_y - last_y_d) / control_duration_;
        double v_x_dot = (v_x - last_vx) / control_duration_;
        double v_y_dot = (v_y - last_vy) / control_duration_;
        // Common VFO
        double h_x = (Kp * e_x) + v_x;
        double h_y = (Kp * e_y) + v_y;

        double theta_a,theta_a_dot;
        double h_x_dot, h_y_dot;

        if (e_mod < 0.1)
        {
            theta_a = theta_d;
            theta_a_dot = 0;
            u2 = 0;
        }
        else
        {
            u2 = h_x * cos(theta) + h_y * sin(theta);
            h_x_dot = (Kp *( v_x -u2 * cos(theta))) + v_x_dot;
            h_y_dot = (Kp *( v_y -u2 * sin(theta))) + v_y_dot;
            // Common VFO
            theta_a = atan2c(sigma * h_y, sigma * h_x);
            theta_a_dot = (h_y_dot * h_x - h_y * h_x_dot) / (pow(h_x, 2) + pow(h_y, 2));
            // End Common VFO
        }
        last_Phi = theta_a;

        // Common VFO
        u1 = Ka * (theta_a) + theta_a_dot;
        // End Common VFO
        std_msgs::msg::Float32MultiArray msg;
        msg.data.push_back(e_x);
        msg.data.push_back(e_y);
        msg.data.push_back(e_mod);
        msg.data.push_back(h_x);
        msg.data.push_back(h_y);
        msg.data.push_back(h_x_dot);
        msg.data.push_back(h_y_dot);
        msg.data.push_back(v_x);
        msg.data.push_back(v_y);
        msg.data.push_back(theta_a);
        msg.data.push_back(theta_a_dot);
        msg.data.push_back(u1);
        msg.data.push_back(u2);


        vfo_param_pub_->publish(msg);
        last_vx = v_x;
        last_vy = v_y;
        last_x_d = e_x;
        last_y_d = e_y;
        RCLCPP_DEBUG(logger_, "E: %.3f,%.3f | e_mod: %.3f, |  h: %.3f,%.3f, | h_dot: %.3f,%.3f |  v: %.3f,%.3f, |  theta_a: %.3f, | theta_a_dot: %.3f, | u: %.3f,%.3f", e_x, e_y, e_mod, h_x, h_y, h_x_dot, h_y_dot, v_x, v_y, theta_a, theta_a_dot, u1, u2);
        return std::array<double, 2>{u1, u2};

    }


    double AGVController::sgn(double v)
    {
    if (v < 0)
        return -1.0;
    if (v > 0)
        return 1.0;
    return 0.0;
    }

    double AGVController::BSP20(double u)
    {
        return u / (std::max(abs(u) / max_speed, 1.0));
    }
    double AGVController::atan2c(double y, double x)
    {
        double current_atan2 = atan2(y, x);
        double last_atan2 = atan2(sin(last_Phi), cos(last_Phi));
        double big_phi = current_atan2 - last_atan2;
        double small_phi = 0;
        if (big_phi > M_PI)
        {
            small_phi = big_phi - 2 * M_PI;
        }
        else if (big_phi < -M_PI)
        {
            small_phi = big_phi + 2 * M_PI;
        }
        else
        {
            small_phi = big_phi;
        }
        return last_Phi + small_phi;
    }


    void AGVController::cleanup()
    {
        RCLCPP_INFO(
            logger_,
            "                                                                           Cleaning up controller: %s of type"
            " VFO::AGVController",
            plugin_name_.c_str());
        global_path_pub_.reset();
        carrot_pub_.reset();
        carrot_arc_pub_.reset();
        vfo_param_pub_.reset();

    }

    void AGVController::activate()
    {
        RCLCPP_INFO(
            logger_,
            "Activating controller: %s of type "
            "VFO::AGVController",
            plugin_name_.c_str());
        global_path_pub_->on_activate();
        carrot_pub_->on_activate();
        carrot_arc_pub_->on_activate();
        vfo_param_pub_->on_activate();
        // Add callback for dynamic parameters
        auto node = node_.lock();
        dyn_params_handler_ = node->add_on_set_parameters_callback(
            std::bind(
                &AGVController::dynamicParametersCallback,
                this, std::placeholders::_1));
    }

    void AGVController::deactivate()
    {
        RCLCPP_INFO(
            logger_,
            "Deactivating controller: %s of type "
            "VFO::AGVController",
            plugin_name_.c_str());
        global_path_pub_->on_deactivate();
        carrot_pub_->on_deactivate();
        carrot_arc_pub_->on_deactivate();
        vfo_param_pub_->on_deactivate();
        dyn_params_handler_.reset();
    }

    std::unique_ptr<geometry_msgs::msg::PointStamped> AGVController::createCarrotMsg(
        const geometry_msgs::msg::PoseStamped &carrot_pose)
    {
        auto carrot_msg = std::make_unique<geometry_msgs::msg::PointStamped>();
        carrot_msg->header = carrot_pose.header;
        carrot_msg->point.x = carrot_pose.pose.position.x;
        carrot_msg->point.y = carrot_pose.pose.position.y;
        carrot_msg->point.z = 0.01; // publish right over map to stand out
        return carrot_msg;
    }

    double AGVController::getLookAheadDistance(
        const geometry_msgs::msg::Twist &speed)
    {
        // If using velocity-scaled look ahead distances, find and clamp the dist
        // Else, use the static look ahead distance
        double lookahead_dist = lookahead_dist_;
        if (use_velocity_scaled_lookahead_dist_)
        {
            lookahead_dist = fabs(speed.linear.x) * lookahead_time_;
            lookahead_dist = std::clamp(lookahead_dist, min_lookahead_dist_, max_lookahead_dist_);
        }

        return lookahead_dist;
    }

    geometry_msgs::msg::TwistStamped AGVController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &speed,
        nav2_core::GoalChecker *goal_checker)
    {
        std::lock_guard<std::mutex> lock_reinit(mutex_);

        // Update for the current goal checker's state
        geometry_msgs::msg::Pose pose_tolerance;
        geometry_msgs::msg::Twist vel_tolerance;
        if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance))
        {
            RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
        }
        else
        {
            goal_dist_tol_ = pose_tolerance.position.x;
        }

        // Transform path to robot base frame
        auto transformed_plan = transformGlobalPlan(pose);

        // Find look ahead distance and point on path and publish
        double lookahead_dist = getLookAheadDistance(speed);

        auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
        carrot_pub_->publish(createCarrotMsg(carrot_pose));
        double linear_vel;
        double angular_vel;
        double angle_to_heading;
        if ((speed.linear.x == 0 and speed.linear.y == 0) and (shouldRotateToGoalHeading(carrot_pose) or shouldRotateToGoalHeading(carrot_pose)) )
        {
            if (shouldRotateToGoalHeading(carrot_pose)) {
                double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
                rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
            } else if (shouldRotateToGoalHeading(carrot_pose)) {
                rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
            }
        }
        else {
            std::array<double, 2> u;
            if (carrot_pose == transformed_plan.poses.back())
            {
                 u = VFO_Point_follow(carrot_pose.pose.position.x, carrot_pose.pose.position.y, tf2::getYaw(carrot_pose.pose.orientation));
            }
            else {
                 u = VFO_Traj_follow(carrot_pose.pose.position.x, carrot_pose.pose.position.y, tf2::getYaw(carrot_pose.pose.orientation));
            }

        linear_vel = BSP20(u[1]);
        angular_vel = BSP20(u[0]);
        RCLCPP_DEBUG(logger_, " Position: (%.3f,%.3f:%.3f) | Carrot: (%.3f,%.3f:%.3f) | Velocity: (%.3f,%.3f) | u: (%.3f,%.3f)", pose.pose.position.x, pose.pose.position.y,  tf2::getYaw(pose.pose.orientation), carrot_pose.pose.position.x, carrot_pose.pose.position.y, tf2::getYaw(carrot_pose.pose.orientation), linear_vel, angular_vel, u[0], u[1]);
        }

        // populate and return message
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header = pose.header;
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    bool AGVController::shouldRotateToPath(
        const geometry_msgs::msg::PoseStamped &carrot_pose, double &angle_to_path)
    {
        // Whether we should rotate robot to rough path heading
        angle_to_path = atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
        return use_rotate_to_heading_ && fabs(angle_to_path) > rotate_to_heading_min_angle_;
    }

    bool AGVController::shouldRotateToGoalHeading(
        const geometry_msgs::msg::PoseStamped &carrot_pose)
    {
        // Whether we should rotate robot to goal heading
        double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
        return use_rotate_to_heading_ && dist_to_goal < goal_dist_tol_;
    }

    void AGVController::rotateToHeading(
        double &linear_vel, double &angular_vel,
        const double &angle_to_path, const geometry_msgs::msg::Twist &curr_speed)
    {
        // Rotate in place using max angular velocity / acceleration possible
        linear_vel = 0.0;
        const double sign = angle_to_path > 0.0 ? 1.0 : -1.0;
        angular_vel = sign * rotate_to_heading_angular_vel_;

        const double &dt = control_duration_;
        const double min_feasible_angular_speed = curr_speed.angular.z - max_angular_accel_ * dt;
        const double max_feasible_angular_speed = curr_speed.angular.z + max_angular_accel_ * dt;
        angular_vel = std::clamp(angular_vel, min_feasible_angular_speed, max_feasible_angular_speed);
    }

    geometry_msgs::msg::Point AGVController::circleSegmentIntersection(
        const geometry_msgs::msg::Point &p1,
        const geometry_msgs::msg::Point &p2,
        double r)
    {
        // Formula for intersection of a line with a circle centered at the origin,
        // modified to always return the point that is on the segment between the two points.
        // https://mathworld.wolfram.com/Circle-LineIntersection.html
        // This works because the poses are transformed into the robot frame.
        // This can be derived from solving the system of equations of a line and a circle
        // which results in something that is just a reformulation of the quadratic formula.
        // Interactive illustration in doc/circle-segment-intersection.ipynb as well as at
        // https://www.desmos.com/calculator/td5cwbuocd
        double x1 = p1.x;
        double x2 = p2.x;
        double y1 = p1.y;
        double y2 = p2.y;

        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr2 = dx * dx + dy * dy;
        double D = x1 * y2 - x2 * y1;

        // Augmentation to only return point within segment
        double d1 = x1 * x1 + y1 * y1;
        double d2 = x2 * x2 + y2 * y2;
        double dd = d2 - d1;

        geometry_msgs::msg::Point p;
        double sqrt_term = std::sqrt(r * r * dr2 - D * D);
        p.x = (D * dy + std::copysign(1.0, dd) * dx * sqrt_term) / dr2;
        p.y = (-D * dx + std::copysign(1.0, dd) * dy * sqrt_term) / dr2;
        return p;
    }

    geometry_msgs::msg::PoseStamped AGVController::getLookAheadPoint(
        const double &lookahead_dist,
        const nav_msgs::msg::Path &transformed_plan)
    {
        // Find the first pose which is at a distance greater than the lookahead distance
        auto goal_pose_it = std::find_if(
            transformed_plan.poses.begin(), transformed_plan.poses.end(), [&](const auto &ps)
            { return hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist; });

        // If the no pose is not far enough, take the last pose
        if (goal_pose_it == transformed_plan.poses.end())
        {
            goal_pose_it = std::prev(transformed_plan.poses.end());
        }
        else if (use_interpolation_ && goal_pose_it != transformed_plan.poses.begin())
        {
            // Find the point on the line segment between the two poses
            // that is exactly the lookahead distance away from the robot pose (the origin)
            // This can be found with a closed form for the intersection of a segment and a circle
            // Because of the way we did the std::find_if, prev_pose is guaranteed to be inside the circle,
            // and goal_pose is guaranteed to be outside the circle.
            auto prev_pose_it = std::prev(goal_pose_it);
            auto point = circleSegmentIntersection(
                prev_pose_it->pose.position,
                goal_pose_it->pose.position, lookahead_dist);
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = prev_pose_it->header.frame_id;
            pose.header.stamp = goal_pose_it->header.stamp;
            pose.pose.position = point;
            return pose;
        }

        return *goal_pose_it;
    }

    bool AGVController::isCollisionImminent(
        const geometry_msgs::msg::PoseStamped &robot_pose,
        const double &linear_vel, const double &angular_vel,
        const double &carrot_dist)
    {
        // Note(stevemacenski): This may be a bit unusual, but the robot_pose is in
        // odom frame and the carrot_pose is in robot base frame.

        // check current point is OK
        if (inCollision(
                robot_pose.pose.position.x, robot_pose.pose.position.y,
                tf2::getYaw(robot_pose.pose.orientation)))
        {
            return true;
        }

        // visualization messages
        nav_msgs::msg::Path arc_pts_msg;
        arc_pts_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
        arc_pts_msg.header.stamp = robot_pose.header.stamp;
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
        pose_msg.header.stamp = arc_pts_msg.header.stamp;

        double projection_time = 0.0;
        if (fabs(linear_vel) < 0.01 && fabs(angular_vel) > 0.01)
        {
            // rotating to heading at goal or toward path
            // Equation finds the angular distance required for the largest
            // part of the robot radius to move to another costmap cell:
            // theta_min = 2.0 * sin ((res/2) / r_max)
            // via isosceles triangle r_max-r_max-resolution,
            // dividing by angular_velocity gives us a timestep.
            double max_radius = costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
            projection_time =
                2.0 * sin((costmap_->getResolution() / 2) / max_radius) / fabs(angular_vel);
        }
        else
        {
            // Normal path tracking
            projection_time = costmap_->getResolution() / fabs(linear_vel);
        }

        const geometry_msgs::msg::Point &robot_xy = robot_pose.pose.position;
        geometry_msgs::msg::Pose2D curr_pose;
        curr_pose.x = robot_pose.pose.position.x;
        curr_pose.y = robot_pose.pose.position.y;
        curr_pose.theta = tf2::getYaw(robot_pose.pose.orientation);

        // only forward simulate within time requested
        int i = 1;
        while (i * projection_time < max_allowed_time_to_collision_up_to_carrot_)
        {
            i++;

            // apply velocity at curr_pose over distance
            curr_pose.x += projection_time * (linear_vel * cos(curr_pose.theta));
            curr_pose.y += projection_time * (linear_vel * sin(curr_pose.theta));
            curr_pose.theta += projection_time * angular_vel;

            // check if past carrot pose, where no longer a thoughtfully valid command
            if (hypot(curr_pose.x - robot_xy.x, curr_pose.y - robot_xy.y) > carrot_dist)
            {
                break;
            }

            // store it for visualization
            pose_msg.pose.position.x = curr_pose.x;
            pose_msg.pose.position.y = curr_pose.y;
            pose_msg.pose.position.z = 0.01;
            arc_pts_msg.poses.push_back(pose_msg);

            // check for collision at the projected pose
            if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta))
            {
                carrot_arc_pub_->publish(arc_pts_msg);
                return true;
            }
        }

        carrot_arc_pub_->publish(arc_pts_msg);

        return false;
    }

    bool AGVController::inCollision(
        const double &x,
        const double &y,
        const double &theta)
    {
        unsigned int mx, my;

        if (!costmap_->worldToMap(x, y, mx, my))
        {
            RCLCPP_WARN_THROTTLE(
                logger_, *(clock_), 30000,
                "The dimensions of the costmap is too small to successfully check for "
                "collisions as far ahead as requested. Proceed at your own risk, slow the robot, or "
                "increase your costmap size.");
            return false;
        }

        double footprint_cost = collision_checker_->footprintCostAtPose(
            x, y, theta, costmap_ros_->getRobotFootprint());
        if (footprint_cost == static_cast<double>(NO_INFORMATION) &&
            costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
        {
            return false;
        }

        // if occupied or unknown and not to traverse unknown space
        return footprint_cost >= static_cast<double>(LETHAL_OBSTACLE);
    }

    double AGVController::costAtPose(const double &x, const double &y)
    {
        unsigned int mx, my;

        if (!costmap_->worldToMap(x, y, mx, my))
        {
            RCLCPP_FATAL(
                logger_,
                "The dimensions of the costmap is too small to fully include your robot's footprint, "
                "thusly the robot cannot proceed further");
            throw nav2_core::PlannerException(
                "AGVController: Dimensions of the costmap are too small "
                "to encapsulate the robot footprint at current speeds!");
        }

        unsigned char cost = costmap_->getCost(mx, my);
        return static_cast<double>(cost);
    }

    double AGVController::approachVelocityScalingFactor(
        const nav_msgs::msg::Path &transformed_path) const
    {
        // Waiting to apply the threshold based on integrated distance ensures we don't
        // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
        double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
        if (remaining_distance < approach_velocity_scaling_dist_)
        {
            auto &last = transformed_path.poses.back();
            // Here we will use a regular euclidean distance from the robot frame (origin)
            // to get smooth scaling, regardless of path density.
            double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
            return distance_to_last_pose / approach_velocity_scaling_dist_;
        }
        else
        {
            return 1.0;
        }
    }

    void AGVController::applyApproachVelocityScaling(
        const nav_msgs::msg::Path &path,
        double &linear_vel) const
    {
        double approach_vel = linear_vel;
        double velocity_scaling = approachVelocityScalingFactor(path);
        double unbounded_vel = approach_vel * velocity_scaling;
        if (unbounded_vel < min_approach_linear_velocity_)
        {
            approach_vel = min_approach_linear_velocity_;
        }
        else
        {
            approach_vel *= velocity_scaling;
        }

        // Use the lowest velocity between approach and other constraints, if all overlapping
        linear_vel = std::min(linear_vel, approach_vel);
    }

    void AGVController::applyConstraints(
        const double &curvature, const geometry_msgs::msg::Twist & /*curr_speed*/,
        const double &pose_cost, const nav_msgs::msg::Path &path, double &linear_vel, double &sign)
    {
        double curvature_vel = linear_vel;
        double cost_vel = linear_vel;

        // limit the linear velocity by curvature
        const double radius = fabs(1.0 / curvature);
        const double &min_rad = regulated_linear_scaling_min_radius_;
        if (use_regulated_linear_velocity_scaling_ && radius < min_rad)
        {
            curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
        }

        // limit the linear velocity by proximity to obstacles
        if (use_cost_regulated_linear_velocity_scaling_ &&
            pose_cost != static_cast<double>(NO_INFORMATION) &&
            pose_cost != static_cast<double>(FREE_SPACE))
        {
            const double inscribed_radius = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
            const double min_distance_to_obstacle = (-1.0 / inflation_cost_scaling_factor_) *
                                                        std::log(pose_cost / (INSCRIBED_INFLATED_OBSTACLE - 1)) +
                                                    inscribed_radius;

            if (min_distance_to_obstacle < cost_scaling_dist_)
            {
                cost_vel *= cost_scaling_gain_ * min_distance_to_obstacle / cost_scaling_dist_;
            }
        }

        // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
        linear_vel = std::min(cost_vel, curvature_vel);
        linear_vel = std::max(linear_vel, regulated_linear_scaling_min_speed_);

        applyApproachVelocityScaling(path, linear_vel);

        // Limit linear velocities to be valid
        linear_vel = std::clamp(fabs(linear_vel), 0.0, desired_linear_vel_);
        linear_vel = sign * linear_vel;
    }

    void AGVController::setPlan(const nav_msgs::msg::Path &path)
    {
        global_plan_ = path;
        e_x0 = 0.0;
        e_y0 = 0.0;
        last_Phi = 0.0;
    }

    void AGVController::setSpeedLimit(
        const double &speed_limit,
        const bool &percentage)
    {
        if (speed_limit == nav2_costmap_2d::NO_SPEED_LIMIT)
        {
            // Restore default value
            desired_linear_vel_ = base_desired_linear_vel_;
        }
        else
        {
            if (percentage)
            {
                // Speed limit is expressed in % from maximum speed of robot
                desired_linear_vel_ = base_desired_linear_vel_ * speed_limit / 100.0;
            }
            else
            {
                // Speed limit is expressed in absolute value
                desired_linear_vel_ = speed_limit;
            }
        }
    }

    nav_msgs::msg::Path AGVController::transformGlobalPlan(
        const geometry_msgs::msg::PoseStamped &pose)
    {
        if (global_plan_.poses.empty())
        {
            throw nav2_core::PlannerException("Received plan with zero length");
        }

        // let's get the pose of the robot in the frame of the plan
        geometry_msgs::msg::PoseStamped robot_pose;
        if (!transformPose(global_plan_.header.frame_id, pose, robot_pose))
        {
            throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
        }

        // We'll discard points on the plan that are outside the local costmap
        double max_costmap_extent = getCostmapMaxExtent();

        auto closest_pose_upper_bound =
            nav2_util::geometry_utils::first_after_integrated_distance(
                global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

        // First find the closest pose on the path to the robot
        // bounded by when the path turns around (if it does) so we don't get a pose from a later
        // portion of the path
        auto transformation_begin =
            nav2_util::geometry_utils::min_by(
                global_plan_.poses.begin(), closest_pose_upper_bound,
                [&robot_pose](const geometry_msgs::msg::PoseStamped &ps)
                {
                    return euclidean_distance(robot_pose, ps);
                });

        // Find points up to max_transform_dist so we only transform them.
        auto transformation_end = std::find_if(
            transformation_begin, global_plan_.poses.end(),
            [&](const auto &pose)
            {
                return euclidean_distance(pose, robot_pose) > max_costmap_extent;
            });

        // Lambda to transform a PoseStamped from global frame to local
        auto transformGlobalPoseToLocal = [&](const auto &global_plan_pose)
        {
            geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
            stamped_pose.header.frame_id = global_plan_.header.frame_id;
            stamped_pose.header.stamp = robot_pose.header.stamp;
            stamped_pose.pose = global_plan_pose.pose;
            transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose);
            transformed_pose.pose.position.z = 0.0;
            return transformed_pose;
        };

        // Transform the near part of the global plan into the robot's frame of reference.
        nav_msgs::msg::Path transformed_plan;
        std::transform(
            transformation_begin, transformation_end,
            std::back_inserter(transformed_plan.poses),
            transformGlobalPoseToLocal);
        transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
        transformed_plan.header.stamp = robot_pose.header.stamp;

        // Remove the portion of the global plan that we've already passed so we don't
        // process it on the next iteration (this is called path pruning)
        global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
        global_path_pub_->publish(transformed_plan);

        if (transformed_plan.poses.empty())
        {
            throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
        }

        return transformed_plan;
    }

    double AGVController::findVelocitySignChange(
        const nav_msgs::msg::Path &transformed_plan)
    {
        // Iterating through the transformed global path to determine the position of the cusp
        for (unsigned int pose_id = 1; pose_id < transformed_plan.poses.size() - 1; ++pose_id)
        {
            // We have two vectors for the dot product OA and AB. Determining the vectors.
            double oa_x = transformed_plan.poses[pose_id].pose.position.x -
                          transformed_plan.poses[pose_id - 1].pose.position.x;
            double oa_y = transformed_plan.poses[pose_id].pose.position.y -
                          transformed_plan.poses[pose_id - 1].pose.position.y;
            double ab_x = transformed_plan.poses[pose_id + 1].pose.position.x -
                          transformed_plan.poses[pose_id].pose.position.x;
            double ab_y = transformed_plan.poses[pose_id + 1].pose.position.y -
                          transformed_plan.poses[pose_id].pose.position.y;

            /* Checking for the existance of cusp, in the path, using the dot product
            and determine it's distance from the robot. If there is no cusp in the path,
            then just determine the distance to the goal location. */
            if ((oa_x * ab_x) + (oa_y * ab_y) < 0.0)
            {
                // returning the distance if there is a cusp
                // The transformed path is in the robots frame, so robot is at the origin
                return hypot(
                    transformed_plan.poses[pose_id].pose.position.x,
                    transformed_plan.poses[pose_id].pose.position.y);
            }
        }

        return std::numeric_limits<double>::max();
    }

    bool AGVController::transformPose(
        const std::string frame,
        const geometry_msgs::msg::PoseStamped &in_pose,
        geometry_msgs::msg::PoseStamped &out_pose) const
    {
        if (in_pose.header.frame_id == frame)
        {
            out_pose = in_pose;
            return true;
        }

        try
        {
            tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
            out_pose.header.frame_id = frame;
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
        }
        return false;
    }

    double AGVController::getCostmapMaxExtent() const
    {
        const double max_costmap_dim_meters = std::max(
            costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
        return max_costmap_dim_meters / 2.0;
    }

    rcl_interfaces::msg::SetParametersResult
    AGVController::dynamicParametersCallback(
        std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        std::lock_guard<std::mutex> lock_reinit(mutex_);

        for (auto parameter : parameters)
        {
            const auto &type = parameter.get_type();
            const auto &name = parameter.get_name();

            if (type == ParameterType::PARAMETER_DOUBLE)
            {
                if (name == plugin_name_ + ".inflation_cost_scaling_factor")
                {
                    if (parameter.as_double() <= 0.0)
                    {
                        RCLCPP_WARN(
                            logger_, "The value inflation_cost_scaling_factor is incorrectly set, "
                                     "it should be >0. Ignoring parameter update.");
                        continue;
                    }
                    inflation_cost_scaling_factor_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".desired_linear_vel")
                {
                    desired_linear_vel_ = parameter.as_double();
                    base_desired_linear_vel_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".lookahead_dist")
                {
                    lookahead_dist_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".max_lookahead_dist")
                {
                    max_lookahead_dist_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".min_lookahead_dist")
                {
                    min_lookahead_dist_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".lookahead_time")
                {
                    lookahead_time_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".rotate_to_heading_angular_vel")
                {
                    rotate_to_heading_angular_vel_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".min_approach_linear_velocity")
                {
                    min_approach_linear_velocity_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".max_allowed_time_to_collision_up_to_carrot")
                {
                    max_allowed_time_to_collision_up_to_carrot_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".cost_scaling_dist")
                {
                    cost_scaling_dist_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".cost_scaling_gain")
                {
                    cost_scaling_gain_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".regulated_linear_scaling_min_radius")
                {
                    regulated_linear_scaling_min_radius_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".transform_tolerance")
                {
                    double transform_tolerance = parameter.as_double();
                    transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
                }
                else if (name == plugin_name_ + ".regulated_linear_scaling_min_speed")
                {
                    regulated_linear_scaling_min_speed_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".max_angular_accel")
                {
                    max_angular_accel_ = parameter.as_double();
                }
                else if (name == plugin_name_ + ".rotate_to_heading_min_angle")
                {
                    rotate_to_heading_min_angle_ = parameter.as_double();
                }
            }
            else if (type == ParameterType::PARAMETER_BOOL)
            {
                if (name == plugin_name_ + ".use_velocity_scaled_lookahead_dist")
                {
                    use_velocity_scaled_lookahead_dist_ = parameter.as_bool();
                }
                else if (name == plugin_name_ + ".use_regulated_linear_velocity_scaling")
                {
                    use_regulated_linear_velocity_scaling_ = parameter.as_bool();
                }
                else if (name == plugin_name_ + ".use_cost_regulated_linear_velocity_scaling")
                {
                    use_cost_regulated_linear_velocity_scaling_ = parameter.as_bool();
                }
                else if (name == plugin_name_ + ".use_rotate_to_heading")
                {
                    if (parameter.as_bool() && allow_reversing_)
                    {
                        RCLCPP_WARN(
                            logger_, "Both use_rotate_to_heading and allow_reversing "
                                     "parameter cannot be set to true. Rejecting parameter update.");
                        continue;
                    }
                    use_rotate_to_heading_ = parameter.as_bool();
                }
                else if (name == plugin_name_ + ".allow_reversing")
                {
                    if (use_rotate_to_heading_ && parameter.as_bool())
                    {
                        RCLCPP_WARN(
                            logger_, "Both use_rotate_to_heading and allow_reversing "
                                     "parameter cannot be set to true. Rejecting parameter update.");
                        continue;
                    }
                    allow_reversing_ = parameter.as_bool();
                }
            }
        }

        result.successful = true;
        return result;
    }

} // namespace nav2_VFO

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(agv_controller::AGVController, nav2_core::Controller)
