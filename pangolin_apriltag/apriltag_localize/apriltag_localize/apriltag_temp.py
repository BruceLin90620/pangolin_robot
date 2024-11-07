import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from isaac_ros_visual_slam_interfaces.srv import SetSlamPose
import numpy as np
from transforms3d import euler, quaternions
import yaml

class AprilTagLocalization(Node):
    def __init__(self):
        super().__init__('apriltag_localization')
        
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.tag_callback,
            10)
        
        self.load_tag_poses()
        self.vslam_client = self.create_client(SetSlamPose, 'visual_slam/set_slam_pose')

    def load_tag_poses(self):
        tag_poses_file = self.declare_parameter('tag_poses_file', '/home/pangolin/pangolin_ws/src/pangolin_robot/pangolin_apriltag/apriltag_localize/config/tag_poses.yaml').value
        with open(tag_poses_file, 'r') as file:
            self.map_tag_info = yaml.safe_load(file)['tag_info']

    def tag_callback(self, msg):
        if not msg.detections:
            self.get_logger().warn("No tag detections received")
            return
        
        detection = msg.detections[0]
        detected_tag_id = detection.id
        detected_tag_pose = detection.pose.pose.pose

        if detected_tag_id not in self.map_tag_info:
            self.get_logger().warn(f"Unknown tag ID: {detected_tag_id}")
            return

        map_tag_pos = self.map_tag_info[detected_tag_id]
        detect_tag_dis = np.hypot(detected_tag_pose.position.z, detected_tag_pose.position.x)

        if detect_tag_dis < 0.7:
            # self.get_logger().info(f"Tag distance: {detect_tag_dis:.2f}")
            robot_pose = self.calculate_robot_pose(detected_tag_pose, map_tag_pos)
            # self.get_logger().info(f"Calculated robot pose: {robot_pose}")
            self.send_pose_update(robot_pose)

    def calculate_robot_pose(self, tag_pose, tag_pos_in_map):
        # Convert quaternion to Euler angles
        tag_pose_euler = euler.quat2euler([tag_pose.orientation.w, tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z])
        tag_pose_in_map_ori = tag_pos_in_map[5]
        # Calculate new orientation
        q = euler.euler2quat(0, 0, tag_pose_euler[1] + tag_pose_in_map_ori, 'sxyz')

        pose = Pose()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q[1], q[2], q[3], q[0]

        # Calculate position based on tag orientation in map
        if tag_pose_in_map_ori == 0:
            pose.position.x = -tag_pose.position.z + tag_pos_in_map[0]
            pose.position.y = tag_pose.position.x + tag_pos_in_map[1]
        elif tag_pose_in_map_ori == 1.57:  # Approximately pi/2
            pose.position.x = -tag_pose.position.x + tag_pos_in_map[0]
            pose.position.y = -tag_pose.position.z + tag_pos_in_map[1]
        elif tag_pose_in_map_ori == -1.57:  # Approximately -pi/2
            pose.position.x = tag_pose.position.x + tag_pos_in_map[0]
            pose.position.y = tag_pose.position.z + tag_pos_in_map[1]
        elif tag_pose_in_map_ori == 3.14:  # Approximately pi
            pose.position.x = tag_pose.position.z + tag_pos_in_map[0]
            pose.position.y = -tag_pose.position.x + tag_pos_in_map[1]
        
        pose.position.z = 0.0  # Assume 2D navigation
        return pose

    def send_pose_update(self, pose):
        req_pose = SetSlamPose.Request()
        req_pose.pose = pose
        self.vslam_client.call_async(req_pose)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()