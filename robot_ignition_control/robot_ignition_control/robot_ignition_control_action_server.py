from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, server
from rclpy.node import Node
import time
from math import sqrt
from nav2_simple_commander.robot_navigator import BasicNavigator


class FollowJointTrajectoryActionServer(Node):

    def __init__(self):
        super().__init__('follow_joint_trajectory_action_server')
        print("Starting action server")
        self.odom = Odometry()
        self._joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/robot_arm_controller/joint_trajectory', 10)
        self._joint_trajectory_publisher2 = self.create_publisher(JointTrajectory, '/hand_controller/joint_trajectory', 10)
        self._cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/robot_controller/follow_joint_trajectory',
            self.execute_callback)
        
        self._action_server2 = ActionServer(
            self,
            FollowJointTrajectory,
            '/hand_controller/follow_joint_trajectory',
            self.execute_callback2)

        self.odom_subscriber = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        print("Initializing nav2 client")
        self.nav = BasicNavigator()
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.nav.setInitialPose(initial_pose)

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def execute_callback(self, goal_handle: server.ServerGoalHandle):
        # ROS_LOGGER = rclpy.logging.get_logger(self.get_logger().name)
        # ROS_LOGGER.info('===============waly=============')
        self.get_logger().info('Executing goal...')
        self.get_logger().info('===============waly=============')

        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory # type: JointTrajectory

        x = len(trajectory.points)
        self.get_logger().info(f'=============== trajectory size = {x} =============')

        self._joint_trajectory_publisher.publish(trajectory)
        multidof_trajectory = goal_handle.request.multi_dof_trajectory # type: MultiDOFJointTrajectory

        x = len(multidof_trajectory.points)
        self.get_logger().info(f'=============== multidof trajectory size = {x} =============')

        if len(multidof_trajectory.points) > 0:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = multidof_trajectory.points[-1].transforms[0].translation.x
            goal_pose.pose.position.y = multidof_trajectory.points[-1].transforms[0].translation.y
            goal_pose.pose.position.z = multidof_trajectory.points[-1].transforms[0].translation.z
            goal_pose.pose.orientation.x = multidof_trajectory.points[-1].transforms[0].rotation.x
            goal_pose.pose.orientation.y = multidof_trajectory.points[-1].transforms[0].rotation.y
            goal_pose.pose.orientation.z = multidof_trajectory.points[-1].transforms[0].rotation.z
            goal_pose.pose.orientation.w = multidof_trajectory.points[-1].transforms[0].rotation.w
            print("Target pose:", goal_pose.pose)
            self.nav.goToPose(goal_pose)
            goal_handle.succeed()
        return result

    def execute_callback2(self, goal_handle: server.ServerGoalHandle):
        # ROS_LOGGER = rclpy.logging.get_logger(self.get_logger().name)
        # ROS_LOGGER.info('===============waly=============')
        self.get_logger().info('Executing goal...')
        self.get_logger().info('===============waly=============')

        result = FollowJointTrajectory.Result()
        trajectory = goal_handle.request.trajectory # type: JointTrajectory

        x = len(trajectory.points)
        self.get_logger().info(f'=============== trajectory size = {x} =============')

        self._joint_trajectory_publisher2.publish(trajectory)
        return result

def main(args=None):
    rclpy.init(args=args)
    follow_joint_trajectory_action_server = FollowJointTrajectoryActionServer()
    rclpy.spin(follow_joint_trajectory_action_server)

if __name__ == '__main__':
    main()
