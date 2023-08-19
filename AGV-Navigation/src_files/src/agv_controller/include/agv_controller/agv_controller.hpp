#ifndef AGV_CONTROLLER__AGV_CONTROLLER_HPP
#define AGV_CONTROLLER__AGV_CONTROLLER_HPP

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <mutex>

#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace agv_controller
{
    class AGVController : public nav2_core::Controller
    {
    public:
        AGVController() = default;
        ~AGVController() = default;

        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

        void activate() override;

        void deactivate() override;

        void cleanup() override;

        // bool isGoalReached() override;

        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity,
            nav2_core::GoalChecker * /*goal_checker*/) override;

        /**
         * @brief nav2_core setPlan - Sets the global plan
         * @param path The global plan
         */
        void setPlan(const nav_msgs::msg::Path &path) override;
        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;

    protected:
        /**
         * @brief Transforms global plan into same frame as pose and clips poses ineligible for lookaheadPoint
         * Points ineligible to be selected as a lookahead point if they are any of the following:
         * - Outside the local_costmap (collision avoidance cannot be assured)
         * @param pose pose to transform
         * @return Path in new frame
         */
        nav_msgs::msg::Path transformGlobalPlan(
            const geometry_msgs::msg::PoseStamped &pose);

        /**
         * @brief Transform a pose to another frame.
         * @param frame Frame ID to transform to
         * @param in_pose Pose input to transform
         * @param out_pose transformed output
         * @return bool if successful
         */
        bool transformPose(
            const std::string frame,
            const geometry_msgs::msg::PoseStamped &in_pose,
            geometry_msgs::msg::PoseStamped &out_pose) const;

        /**
         * @brief Get lookahead distance
         * @param cmd the current speed to use to compute lookahead point
         * @return lookahead distance
         */
        double getLookAheadDistance(const geometry_msgs::msg::Twist &);

        /**
         * @brief Creates a PointStamped message for visualization
         * @param carrot_pose Input carrot point as a PoseStamped
         * @return CarrotMsg a carrot point marker, PointStamped
         */
        std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
            const geometry_msgs::msg::PoseStamped &carrot_pose);

        /**
         * @brief Whether robot should rotate to rough path heading
         * @param carrot_pose current lookahead point
         * @param angle_to_path Angle of robot output relatie to carrot marker
         * @return Whether should rotate to path heading
         */
        bool shouldRotateToPath(
            const geometry_msgs::msg::PoseStamped &carrot_pose, double &angle_to_path);

        /**
         * @brief Whether robot should rotate to final goal orientation
         * @param carrot_pose current lookahead point
         * @return Whether should rotate to goal heading
         */
        bool shouldRotateToGoalHeading(const geometry_msgs::msg::PoseStamped &carrot_pose);

        /**
         * @brief Create a smooth and kinematically smoothed rotation command
         * @param linear_vel linear velocity
         * @param angular_vel angular velocity
         * @param angle_to_path Angle of robot output relatie to carrot marker
         * @param curr_speed the current robot speed
         */
        void rotateToHeading(
            double &linear_vel, double &angular_vel,
            const double &angle_to_path, const geometry_msgs::msg::Twist &curr_speed);

        /**
         * @brief Whether collision is imminent
         * @param robot_pose Pose of robot
         * @param carrot_pose Pose of carrot
         * @param linear_vel linear velocity to forward project
         * @param angular_vel angular velocity to forward project
         * @param carrot_dist Distance to the carrot for PP
         * @return Whether collision is imminent
         */
        bool isCollisionImminent(
            const geometry_msgs::msg::PoseStamped &,
            const double &, const double &,
            const double &);

        /**
         * @brief checks for collision at projected pose
         * @param x Pose of pose x
         * @param y Pose of pose y
         * @param theta orientation of Yaw
         * @return Whether in collision
         */
        bool inCollision(
            const double &x,
            const double &y,
            const double &theta);
        /**
         * @brief Cost at a point
         * @param x Pose of pose x
         * @param y Pose of pose y
         * @return Cost of pose in costmap
         */
        double costAtPose(const double &x, const double &y);

        double approachVelocityScalingFactor(
            const nav_msgs::msg::Path &path) const;

        void applyApproachVelocityScaling(
            const nav_msgs::msg::Path &path,
            double &linear_vel) const;

        /**
         * @brief apply regulation constraints to the system
         * @param linear_vel robot command linear velocity input
         * @param lookahead_dist optimal lookahead distance
         * @param curvature curvature of path
         * @param speed Speed of robot
         * @param pose_cost cost at this pose
         */
        void applyConstraints(
            const double &curvature, const geometry_msgs::msg::Twist &speed,
            const double &pose_cost, const nav_msgs::msg::Path &path,
            double &linear_vel, double &sign);

        /**
         * @brief Find the intersection a circle and a line segment.
         * This assumes the circle is centered at the origin.
         * If no intersection is found, a floating point error will occur.
         * @param p1 first endpoint of line segment
         * @param p2 second endpoint of line segment
         * @param r radius of circle
         * @return point of intersection
         */
        static geometry_msgs::msg::Point circleSegmentIntersection(
            const geometry_msgs::msg::Point &p1,
            const geometry_msgs::msg::Point &p2,
            double r);

        /**
         * @brief Get lookahead point
         * @param lookahead_dist Optimal lookahead distance
         * @param path Current global path
         * @return Lookahead point
         */
        geometry_msgs::msg::PoseStamped getLookAheadPoint(const double &, const nav_msgs::msg::Path &);

        /**
         * @brief checks for the cusp position
         * @param pose Pose input to determine the cusp position
         * @return robot distance from the cusp
         */
        double findVelocitySignChange(const nav_msgs::msg::Path &transformed_plan);

        /**
         * Get the greatest extent of the costmap in meters from the center.
         * @return max of distance from center in meters to edge of costmap
         */
        double getCostmapMaxExtent() const;

        /**
         * @brief Callback executed when a parameter change is detected
         * @param event ParameterEvent message
         */
        rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::string plugin_name_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        nav2_costmap_2d::Costmap2D *costmap_;
        rclcpp::Logger logger_{rclcpp::get_logger("VFO")};
        rclcpp::Clock::SharedPtr clock_;

        double desired_linear_vel_, base_desired_linear_vel_;
        double lookahead_dist_;
        double rotate_to_heading_angular_vel_;
        double max_lookahead_dist_;
        double min_lookahead_dist_;
        double lookahead_time_;
        bool use_velocity_scaled_lookahead_dist_;
        tf2::Duration transform_tolerance_;
        double min_approach_linear_velocity_;
        double approach_velocity_scaling_dist_;
        double control_duration_;
        double max_allowed_time_to_collision_up_to_carrot_;
        bool use_collision_detection_;
        bool use_regulated_linear_velocity_scaling_;
        bool use_cost_regulated_linear_velocity_scaling_;
        double cost_scaling_dist_;
        double cost_scaling_gain_;
        double inflation_cost_scaling_factor_;
        double regulated_linear_scaling_min_radius_;
        double regulated_linear_scaling_min_speed_;
        bool use_rotate_to_heading_;
        double max_angular_accel_;
        double rotate_to_heading_min_angle_;
        double goal_dist_tol_;
        bool allow_reversing_;
        double max_robot_pose_search_dist_;
        bool use_interpolation_;

        nav_msgs::msg::Path global_plan_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>>
            carrot_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> carrot_arc_pub_;
        std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
            collision_checker_;

        // Dynamic parameters handler
        std::mutex mutex_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

         // VFO
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32MultiArray>> vfo_param_pub_;

        double last_Phi,v_d, Kp, Ka, max_speed;
        double carrot_x, carrot_y, carrot_theta, carrot_dist;
        double e_x0 = 0.0, e_y0 = 0.0;
        double u1,u2;
        double e_x0_d = 0.0;
        double last_x_d, last_y_d, last_vx, last_vy;
        double atan2c(double y, double x);
        double BSP20(double u);
        // std::array<double, 2> VFO(double x, double y, double theta, double F, double Fx, double Fy, double Fxx, double Fyy, double Fxy);
        std::array<double, 2> VFO_Point_follow(const double e_x, const double e_y, const double theta_d);
        std::array<double, 2> VFO_Traj_follow(const double e_x, const double e_y, const double theta_d);
        double sgn(double v);


    };

} // namespace agv_controller

#endif // AGV_CONTROLLER__AGV_CONTROLLER_HPP
