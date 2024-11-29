import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

from isaac_ros_visual_slam_interfaces.action import LoadMapAndLocalize

class Nav2VslamLocalize(Node):

    def __init__(self):
        super().__init__('nav2_vslam_localize')
        self.vslam_localize_action_client = ActionClient(self, LoadMapAndLocalize, '/visual_slam/load_map_and_localize')
        self.init_pose_subscription = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.init_pose_callback, 1)

    def send_goal(self, init_pose_msg):
        self.get_logger().info('get goal')
        goal_msg = LoadMapAndLocalize.Goal()
        goal_msg.map_url = "/home/pangolin/pangolin_ws/pangolin_robot/pangolin_visual_slam/isaac_ros_visual_slam/isaac_ros_visual_slam/map"

        goal_msg.localize_near_point.x = init_pose_msg.pose.pose.position.x
        goal_msg.localize_near_point.y = init_pose_msg.pose.pose.position.y
        goal_msg.localize_near_point.z = init_pose_msg.pose.pose.position.z

        self.get_logger().info('waiting for server...')
        self.vslam_localize_action_client.wait_for_server()

        self.get_logger().info(f'send goal {goal_msg.localize_near_point}')
        self._send_goal_future = self.vslam_localize_action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Goal accepted by server')
        else:
            self.get_logger().info('Goal rejected by server')


    def init_pose_callback(self, msg):

        self.send_goal(msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = Nav2VslamLocalize()

    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
