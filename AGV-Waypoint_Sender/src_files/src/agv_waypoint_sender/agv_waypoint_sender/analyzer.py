from datetime import datetime
import os
import threading
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

class Analyzer(Node):
    def __init__(self):
        super().__init__("Analyzer")
        self.declare_parameter("test_name", "Params not set")
        self.test_name = self.get_parameter("test_name").value
        with open(f"/results/feedback_{self.test_name}.txt", "w") as f:
            f.write("LinearX,LinearY,AngularZ\n")
        self.last_vfo = ""

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_subscription(Float32MultiArray, "/vfo", self.vfo_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_pose_callback, 10)

    def cmd_vel_callback(self, msg):
        with open(f"/results/feedback_{self.test_name}.txt", "a") as f:
            f.write(f"{msg.linear.x},{msg.angular.z},{self.last_vfo}\n")

    def vfo_callback(self, msg):
        self.last_vfo = ""
        for data in msg.data:
            self.last_vfo += f"{data},"

    def odom_callback(self, msg):
        with open(f"/results/odom_{self.test_name}.txt", "a") as f:
            f.write(f"{msg.pose.pose.position.x},{msg.pose.pose.position.y}\n")
            
    def amcl_pose_callback(self, msg):
        with open(f"/results/amcl_{self.test_name}.txt", "a") as f:
            f.write(f"{msg.pose.pose.position.x},{msg.pose.pose.position.y}\n")


def main(args=None):
    rclpy.init(args=args)
    goal_publisher = Analyzer()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
