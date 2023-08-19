from datetime import datetime
import os
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

WAYPOINTS = [
    [0.8, -18.5, 0, 0, 0, 0.02, 1.0],
    [16.0, -18.0, 0, 0, 0, 0.01, 1.0],
    # [4.7, -22.8, 0, 0, 0, 1.0, 0.4],
    # [-14.5, -18.8, 0, 0, 0, 0.7, 0.7],
    # [-3.13, -3.40, 0, 0, 0, 0.015, 1.0],
    # [-11.0, -17.5, 0, 0, 0, -0.7, 0.7],
]


class GoalPublisher(Node):
    def __init__(self):
        super().__init__("GoalPublisher")
        self.declare_parameter("test_name", "Params not set")
        self.test_name = self.get_parameter("test_name").value
        self.next_target_publisher = self.create_publisher(
            PoseStamped, "/next_target", 10
        )
        # Publish list of all targets to /all_targets
        self.all_targets_publisher = self.create_publisher(Path, "/all_targets", 10)
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.goal_poses = []
        self.load_points()
        self.publish_points()
        self.point_loop()

    def publish_points(self):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = self.goal_poses
        self.all_targets_publisher.publish(path)

    def point_loop(self):
        time_array = []
        failed = 0
        total = 0

        # Main loop for navigation
        for goal_pose in self.goal_poses:
            self.get_logger().info(f"Sending goal: {goal_pose}")
            self.navigator.goToPose(goal_pose)
            self.send_pose(
                goal_pose.pose.position.x,
                goal_pose.pose.position.y,
                goal_pose.pose.position.z,
                goal_pose.pose.orientation.x,
                goal_pose.pose.orientation.y,
                goal_pose.pose.orientation.z,
                goal_pose.pose.orientation.w,
            )
            self.publish_points()

            nav_start = self.navigator.get_clock().now()

            i = 0
            while not self.navigator.isTaskComplete():
                self.publish_points()

                i = i + 1
                feedback = self.navigator.getFeedback()
                now = self.navigator.get_clock().now()

                # if feedback and i % 15 == 0:
                #     self.get_logger().debug(
                #         "Estimated time of arrival: "
                #         + "{0:.0f}".format(
                #             Duration.from_msg(
                #                 feedback.estimated_time_remaining
                #             ).nanoseconds
                #             / 1e9
                #         )
                #         + " seconds."
                #     )

                if now - nav_start > Duration(seconds=150.0):
                    self.navigator.cancelTask()
                    self.get_logger().info("Canceling task")
                    break
            result = self.navigator.getResult()

            if result is not None:
                if result == TaskResult.SUCCEEDED:
                    time_array.append(now - nav_start)
                    self.get_logger().info(f"Goal succeeded! {result}")
                elif result == TaskResult.CANCELED:
                    self.get_logger().error("Goal was canceled!")
                    self.end()
                elif result == TaskResult.FAILED:
                    self.get_logger().error("Goal failed!")
                    self.end()

                else:
                    failed += 1
                    self.get_logger().error("Goal has an invalid return status!")

        # Save results to file
        line = f"{self.test_name},"
        for t in time_array:
            line += f"{t.nanoseconds},"
            total += t.nanoseconds
        line += f"{total}"
        self.navigator.lifecycleShutdown()
        self.navigator.destroy_node()
        exit(f"Test results: {line} Test results:")

    def end(self):
        self.navigator.lifecycleShutdown()
        self.navigator.destroy_node()
        exit(f"Test results: FAILED Test results:")

    def load_points(self):
        for point in WAYPOINTS:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(point[0])
            goal_pose.pose.position.y = float(point[1])
            goal_pose.pose.position.z = float(point[2])
            goal_pose.pose.orientation.x = float(point[3])
            goal_pose.pose.orientation.y = float(point[4])
            goal_pose.pose.orientation.z = float(point[5])
            goal_pose.pose.orientation.w = float(point[6])
            self.goal_poses.append(goal_pose)

    def send_pose(self, x, y, z, ox, oy, oz, ow):
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.header.stamp = self.get_clock().now().to_msg()
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.position.z = z
        self.goal.pose.orientation.x = ox
        self.goal.pose.orientation.y = oy
        self.goal.pose.orientation.z = oz
        self.goal.pose.orientation.w = ow
        self.next_target_publisher.publish(self.goal)
        self.get_logger().info(f"Publishing: X: {x}, Y: {y}, R: {ow}")


def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
